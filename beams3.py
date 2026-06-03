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
        s = f"{cm * 10.0:.2f}".rstrip("0").rstrip(".")
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

    def make_ibeam(
        comp, face, beam_len, fw, th, ft, wt, base_name, flange_dir, height_dir
    ):
        sketch = comp.sketches.add(face)
        sketch.name = base_name + "_IBeam_Profile"

        origin = face.centroid
        c = sketch.modelToSketchSpace(origin)
        cx, cy = c.x, c.y

        def world_dir_to_sketch(d):
            pt = adsk.core.Point3D.create(
                origin.x + d.x, origin.y + d.y, origin.z + d.z
            )
            sk = sketch.modelToSketchSpace(pt)
            return sk.x - cx, sk.y - cy

        ux, uy = world_dir_to_sketch(flange_dir)
        vx, vy = world_dir_to_sketch(height_dir)

        def p(u, v):
            return adsk.core.Point3D.create(
                cx + u * ux + v * vx,
                cy + u * uy + v * vy,
                0,
            )

        lines = sketch.sketchCurves.sketchLines
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
    # 2. ASK ORIENTATION FIRST (Right after body selection)
    # -------------------------------------------------------------------------
    orientation = ui.messageBox(
        "Beam orientation?\n\n"
        "Yes  →  Wide  (flanges along the wider cross-section axis)\n"
        "No   →  Tall  (flanges along the narrower cross-section axis)",
        "Beam Orientation",
        adsk.core.MessageBoxButtonTypes.YesNoButtonType,
        adsk.core.MessageBoxIconTypes.QuestionIconType,
    )
    # If the user closes the message box window without clicking yes/no
    if orientation == adsk.core.DialogResults.DialogCancel:
        return

    orient_wide = orientation == adsk.core.DialogResults.DialogYes

    # -------------------------------------------------------------------------
    # 3. Derive cross-section defaults based on OBB and chosen orientation
    # -------------------------------------------------------------------------
    obb0 = selected_body.orientedMinimumBoundingBox
    all_dims = sorted(
        [
            (obb0.length, obb0.lengthDirection),
            (obb0.width, obb0.widthDirection),
            (obb0.height, obb0.heightDirection),
        ],
        key=lambda x: x[0],
    )

    cs_small_val, _ = all_dims[0]  # smaller cross-section dim
    cs_large_val, _ = all_dims[1]  # larger  cross-section dim

    # Dynamically shift default inputs depending on user's orientation choice
    if orient_wide:
        default_w = cs_large_val
        default_h = cs_small_val
    else:
        default_w = cs_small_val
        default_h = cs_large_val

    thickness_default = cs_small_val / 6.0

    # -------------------------------------------------------------------------
    # 4. Prompt for Dimensions with smart dynamic defaults
    # -------------------------------------------------------------------------
    flange_w = ask("Flange width", fmt(default_w))
    if flange_w is None:
        return

    total_h = ask("Total height", fmt(default_h))
    if total_h is None:
        return

    flange_t = ask("Flange thickness", fmt(thickness_default))
    if flange_t is None:
        return

    web_t = ask("Web thickness", fmt(flange_t))  # default = same as flange
    if web_t is None:
        return

    fw = flange_w / 2.0
    th = total_h / 2.0
    ft = flange_t
    wt = web_t / 2.0

    # -------------------------------------------------------------------------
    # 5. Match sibling bodies
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
    # 6. Create beams
    # -------------------------------------------------------------------------
    created = []

    for body in matching_bodies:
        try:
            obb = body.orientedMinimumBoundingBox
            dims = sorted(
                [
                    (obb.length, obb.lengthDirection),
                    (obb.width, obb.widthDirection),
                    (obb.height, obb.heightDirection),
                ],
                key=lambda x: x[0],
            )

            beam_len, beam_dir = dims[2]

            if orient_wide:
                _, height_dir = dims[0]  # smaller → height
                _, flange_dir = dims[1]  # larger  → flange width
            else:
                _, flange_dir = dims[0]  # smaller → flange width
                _, height_dir = dims[1]  # larger  → height

            end_face = best_end_face(body, beam_dir)
            if not end_face:
                continue

            new_body = make_ibeam(
                rootComp,
                end_face,
                beam_len,
                fw,
                th,
                ft,
                wt,
                body.name,
                flange_dir,
                height_dir,
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
