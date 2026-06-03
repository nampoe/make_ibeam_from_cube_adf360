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

    def make_ibeam(comp, face, beam_len, fw, th, ft, wt, base_name):
        sketch = comp.sketches.add(face)
        lines = sketch.sketchCurves.sketchLines
        center = sketch.modelToSketchSpace(face.centroid)

        cx = center.x
        cy = center.y

        def p(x, y):
            return adsk.core.Point3D.create(cx + x, cy + y, 0)

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

        largest_profile = None
        largest_area = -1

        for i in range(sketch.profiles.count):
            prof = sketch.profiles.item(i)
            area = prof.areaProperties(
                adsk.fusion.CalculationAccuracy.LowCalculationAccuracy
            ).area
            if area > largest_area:
                largest_area = area
                largest_profile = prof

        if not largest_profile:
            sketch.deleteMe()
            return None

        ext = comp.features.extrudeFeatures.addSimple(
            largest_profile,
            adsk.core.ValueInput.createByReal(-beam_len),
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
        )

        body = ext.bodies.item(0)
        body.name = base_name + "_IBeam"
        return body

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
    # UI Selection Alternative (Bulletproof compared to API folder mapping)
    # -------------------------------------------------------------------------
    try:
        selection = ui.selectEntity(
            "Select a sample body inside your target folder:", "SolidBodies"
        )
        if not selection:
            return
        selected_body = adsk.fusion.BRepBody.cast(selection.entity)
    except:
        return  # User cancelled selection

    # Find the parent component container or root
    parent_comp = selected_body.parentComponent
    if not parent_comp:
        parent_comp = rootComp

    # -------------------------------------------------------------------------
    # Match Sibling Bodies Natively
    # -------------------------------------------------------------------------
    matching_bodies = []

    # We strip down the naming convention because your bodies are cleanly named:
    # "Länsi Tukijalka Palikka...", "Etelä Tukijalka Palikka..."
    # We look for common structural keywords like "Tukijalka", "Poikkipilari", etc.
    base_identifier = ""
    if "tukijalka" in selected_body.name.lower():
        base_identifier = "tukijalka"
    elif "poikkipilari" in selected_body.name.lower():
        base_identifier = "poikkipilari"
    elif "ristiputki" in selected_body.name.lower():
        base_identifier = "ristiputki"
    else:
        # Fallback to the first word of whatever you clicked
        base_identifier = selected_body.name.split()[0].lower()

    # Scan all bodies in that component area for matches
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

            new_body = make_ibeam(
                rootComp, end_face, beam_len, fw, th, ft, wt, body.name
            )
            if new_body:
                created.append(new_body.name)

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
        f"Sibling bodies identified: {len(matching_bodies)}",
        f"Beams successfully created: {len(created)}",
        "",
    ]
    report.extend(created)
    ui.messageBox("\n".join(report))
