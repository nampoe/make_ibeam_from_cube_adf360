import math

import adsk.core
import adsk.fusion


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface

    design = adsk.fusion.Design.cast(app.activeProduct)
    if not design:
        ui.messageBox("No active Fusion design.")
        return

    rootComp = design.rootComponent
    uMgr = design.unitsManager
    lu = uMgr.defaultLengthUnits

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------
    def ask(prompt, default):
        while True:
            value, cancelled = ui.inputBox(prompt, "I/H-Beam Dimensions", default)
            if cancelled:
                return None
            if uMgr.isValidExpression(value, lu):
                return uMgr.evaluateExpression(value, lu)  # Returns internal CM units
            ui.messageBox(f'"{value}" is not a valid length.\nExample: "100 mm"')

    def best_end_face(body, beam_dir):
        best_face = None
        best_dot = -1

        for face in body.faces:
            if face.geometry.surfaceType != adsk.core.SurfaceTypes.PlaneSurfaceType:
                continue

            dot = abs(face.geometry.normal.dotProduct(beam_dir))
            if dot > best_dot:
                best_dot = dot
                best_face = face

        return best_face

    def make_ibeam(comp, face, beam_len, fw, th, ft, wt, base_name, target_body):
        sketch = comp.sketches.add(face)
        lines = sketch.sketchCurves.sketchLines
        center = sketch.modelToSketchSpace(face.centroid)

        cx = center.x
        cy = center.y

        def p(x, y):
            return adsk.core.Point3D.create(cx + x, cy + y, 0)

        # Build sketch profile points relative to the face centroid
        pts = [
            p(-fw, th),
            p(fw, th),
            p(fw, th - ft),
            p(wt, th - ft),
            p(wt, -th + ft),
            p(fw, -th + ft),
            p(fw, -th),
            p(-fw, -th),
            p(-fw, -th + ft),
            p(-wt, -th + ft),
            p(-wt, th - ft),
            p(-fw, th - ft),
        ]

        for i in range(len(pts)):
            lines.addByTwoPoints(pts[i], pts[(i + 1) % len(pts)])

        # ---------------------------------------------------------------------
        # FIX 1: Locate the precise interior profile matching the "I" shape
        # ---------------------------------------------------------------------
        ibeam_profile = None
        center_point_2d = adsk.core.Point2D.create(cx, cy)

        for i in range(sketch.profiles.count):
            prof = sketch.profiles.item(i)
            containment = prof.containmentType(center_point_2d)
            if (
                containment
                == adsk.fusion.ProfileContainmentTypes.ProfileInteriorContainment
            ):
                ibeam_profile = prof
                break

        if not ibeam_profile:
            sketch.deleteMe()
            return None

        # ---------------------------------------------------------------------
        # FIX 2: Isolate the extrusion operation exclusively to our duplicated body
        # ---------------------------------------------------------------------
        extrudes = comp.features.extrudeFeatures
        ext_input = extrudes.createInput(
            ibeam_profile, adsk.fusion.FeatureOperations.IntersectFeatureOperation
        )

        # Explicitly restrict target bodies to block out global timeline artifacts
        ext_input.targetBodies = [target_body]

        # Extrude along the inverted normal direction
        distance = adsk.core.ValueInput.createByReal(-beam_len)
        ext_input.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(distance),
            adsk.fusion.ExtentDirections.PositiveExtentDirection,
        )

        ext = extrudes.add(ext_input)

        if ext.bodies.count > 0:
            body = ext.bodies.item(0)
            body.name = base_name + "_IBeam"
            return body

        return None

    # -------------------------------------------------------------------------
    # Get Dimensions
    # -------------------------------------------------------------------------
    flange_w = ask("Flange width", "100 mm")
    if flange_w is None:
        return

    total_h = ask("Total height", "150 mm")
    if total_h is None:
        return

    flange_t = ask("Flange thickness", "10 mm")
    if flange_t is None:
        return

    web_t = ask("Web thickness", "8 mm")
    if web_t is None:
        return

    fw = flange_w / 2.0
    th = total_h / 2.0
    ft = flange_t
    wt = web_t / 2.0

    # -------------------------------------------------------------------------
    # UI Selection
    # -------------------------------------------------------------------------
    try:
        selection = ui.selectEntity(
            "Select a sample body inside your target folder:", "SolidBodies"
        )
        if not selection:
            return
        selected_body = adsk.fusion.BRepBody.cast(selection.entity)
    except:
        return

    parent_comp = (
        selected_body.parentComponent if selected_body.parentComponent else rootComp
    )

    # -------------------------------------------------------------------------
    # Match Sibling Bodies
    # -------------------------------------------------------------------------
    matching_bodies = []
    base_identifier = ""
    if "tukijalka" in selected_body.name.lower():
        base_identifier = "tukijalka"
    elif "poikkipilari" in selected_body.name.lower():
        base_identifier = "poikkipilari"
    elif "ristiputki" in selected_body.name.lower():
        base_identifier = "ristiputki"
    else:
        base_identifier = selected_body.name.split()[0].lower()

    for i in range(parent_comp.bRepBodies.count):
        body = parent_comp.bRepBodies.item(i)
        if body.isSolid and base_identifier in body.name.lower():
            matching_bodies.append(body)

    # -------------------------------------------------------------------------
    # Create Beams
    # -------------------------------------------------------------------------
    created = []

    for body in matching_bodies:
        try:
            obb = body.orientedMinimumBoundingBox

            dims = [
                (obb.length, obb.lengthDirection),
                (obb.width, obb.widthDirection),
                (obb.height, obb.heightDirection),
            ]

            beam_len, beam_dir = max(dims, key=lambda x: x[0])
            end_face = best_end_face(body, beam_dir)

            if not end_face:
                continue

            # -----------------------------------------------------------------
            # FIX 3: Copy the original body first to protect the source geometry
            # -----------------------------------------------------------------
            body_duplicate = body.copyToComponent(parent_comp)

            # The original body stays safe; we modify only the clone
            new_body = make_ibeam(
                rootComp, end_face, beam_len, fw, th, ft, wt, body.name, body_duplicate
            )

            if new_body:
                created.append(new_body.name)
            else:
                # Clean up the unused clone if the beam creation fails
                body_duplicate.deleteMe()

        except:
            pass

    # -------------------------------------------------------------------------
    # Report Execution Result
    # -------------------------------------------------------------------------
    try:
        app.activeViewport.fit()
    except:
        pass

    report = [
        f"Matched keyword: '{base_identifier}'",
        f"Original solid bodies preserved: {len(matching_bodies)}",
        f"New I-Beams created in place: {len(created)}",
        "",
    ]
    report.extend(created)
    ui.messageBox("\n".join(report))
