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
                return uMgr.evaluateExpression(value, lu)
            ui.messageBox(f'"{value}" is not a valid length.\nExample: "100 mm"')

    def fmt(cm):
        """Internal cm → rounded mm string for input-box defaults."""
        mm = cm * 10.0
        # Round to nearest 0.1 mm; strip trailing zeros for readability
        s = f"{mm:.2f}".rstrip("0").rstrip(".")
        return f"{s} mm"

    def best_end_face(body, beam_dir):
        best_face, best_dot = None, -1.0
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
        sketch.name = base_name + "_IBeam_Profile"
        lines = sketch.sketchCurves.sketchLines
        center = sketch.modelToSketchSpace(face.centroid)
        cx, cy = center.x, center.y

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

        # Pick the smallest-area profile that contains the web centroid.
        # Sketching on a face projects the face boundary into the sketch,
        # producing an outer-rectangle profile alongside the I-beam profile —
        # both contain the centroid, so we disambiguate by area.
        ibeam_profile = None
        min_area = float("inf")
        center_2d = adsk.core.Point2D.create(cx, cy)
        Interior = adsk.fusion.ProfileContainmentTypes.ProfileInteriorContainment

        for i in range(sketch.profiles.count):
            prof = sketch.profiles.item(i)
            if prof.containmentType(center_2d) == Interior:
                area = prof.areaProperties().area
                if area < min_area:
                    min_area = area
                    ibeam_profile = prof

        if not ibeam_profile:
            sketch.deleteMe()
            return None

        original_body = face.body
        try:
            extrudes = comp.features.extrudeFeatures
            ext_input = extrudes.createInput(
                ibeam_profile,
                adsk.fusion.FeatureOperations.IntersectFeatureOperation,
            )
            ext_input.setOneSideExtent(
                adsk.fusion.DistanceExtentDefinition.create(
                    adsk.core.ValueInput.createByReal(-beam_len)
                ),
                adsk.fusion.ExtentDirections.PositiveExtentDirection,
            )
            ext_input.participantBodies = [original_body]
            extrudes.add(ext_input)

            original_body.name = base_name + "_IBeam"
            return original_body
        except Exception:
            return None

    # -------------------------------------------------------------------------
    # 1. Select body first so we can read its dimensions for defaults
    # -------------------------------------------------------------------------
    try:
        selection = ui.selectEntity(
            "Select a sample body inside your target folder:", "SolidBodies"
        )
        if not selection:
            return
        selected_body = adsk.fusion.BRepBody.cast(selection.entity)
    except Exception:
        return

    # -------------------------------------------------------------------------
    # 2. Derive cross-section defaults from the selected body's OBB
    #
    #    Sort the three OBB dimensions: longest = beam axis,
    #    the remaining two are the cross-section (width × height).
    #    Flange width  → larger  cross-section dim
    #    Total height  → smaller cross-section dim   (swap in inputbox if needed)
    #    Flange/web t  → smaller_dim / 6  (equal by default, so web ≡ side thickness)
    # -------------------------------------------------------------------------
    obb = selected_body.orientedMinimumBoundingBox
    all_dims = sorted([obb.length, obb.width, obb.height])  # ascending
    # all_dims[2] = beam length, [0] and [1] = cross-section
    cs_a, cs_b = all_dims[0], all_dims[1]  # smaller, larger
    thickness_default = cs_a / 6.0  # flange_t == web_t

    # -------------------------------------------------------------------------
    # 3. Prompt for dimensions — pre-filled from the body
    # -------------------------------------------------------------------------
    flange_w = ask("Flange width", fmt(cs_b))
    if flange_w is None:
        return

    total_h = ask("Total height", fmt(cs_a))
    if total_h is None:
        return

    flange_t = ask("Flange thickness", fmt(thickness_default))
    if flange_t is None:
        return

    # Default web thickness == flange thickness ("center same size as sides")
    web_t = ask("Web thickness", fmt(flange_t))
    if web_t is None:
        return

    fw = flange_w / 2.0
    th = total_h / 2.0
    ft = flange_t
    wt = web_t / 2.0

    # -------------------------------------------------------------------------
    # 4. Match sibling bodies
    # -------------------------------------------------------------------------
    parent_comp = selected_body.parentComponent or rootComp

    if "tukijalka" in selected_body.name.lower():
        base_identifier = "tukijalka"
    elif "poikkipilari" in selected_body.name.lower():
        base_identifier = "poikkipilari"
    elif "ristiputki" in selected_body.name.lower():
        base_identifier = "ristiputki"
    else:
        base_identifier = selected_body.name.split()[0].lower()

    matching_bodies = [
        parent_comp.bRepBodies.item(i)
        for i in range(parent_comp.bRepBodies.count)
        if parent_comp.bRepBodies.item(i).isSolid
        and base_identifier in parent_comp.bRepBodies.item(i).name.lower()
    ]

    # -------------------------------------------------------------------------
    # 5. Create beams
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
        except Exception:
            pass

    try:
        app.activeViewport.fit()
    except Exception:
        pass

    report = [
        f"Matched keyword: '{base_identifier}'",
        f"Bodies processed: {len(matching_bodies)}",
        f"I-Beams created:  {len(created)}",
        "",
        *created,
    ]
    ui.messageBox("\n".join(report))
