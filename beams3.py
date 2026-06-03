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
        """
        flange_dir : unit Vector3D along the flange width  (OBB cross-section axis)
        height_dir : unit Vector3D along the beam height   (OBB cross-section axis)

        Both are in world space.  We project them into sketch space so the
        I-beam profile follows the body's actual orientation, not the world grid.
        """
        sketch = comp.sketches.add(face)
        sketch.name = base_name + "_IBeam_Profile"

        # ── Map world-space cross-section axes → sketch-space basis vectors ──
        # modelToSketchSpace converts a world Point3D to sketch Point2D.
        # Offsetting the centroid by a unit direction and converting gives us
        # the direction in sketch coordinates.
        origin = face.centroid
        c = sketch.modelToSketchSpace(origin)
        cx, cy = c.x, c.y

        def world_dir_to_sketch(d):
            """Return the 2-D sketch-space vector for world-space unit vector d."""
            pt = adsk.core.Point3D.create(
                origin.x + d.x, origin.y + d.y, origin.z + d.z
            )
            sk = sketch.modelToSketchSpace(pt)
            return sk.x - cx, sk.y - cy

        # u = flange direction (horizontal),  v = height direction (vertical)
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

        # Smallest-area profile that contains the web centroid = the I-shape
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
    #    Sort all three dimensions; the longest is the beam axis.
    #    The two shorter ones are the cross-section (flange_w × total_h).
    #    Flange width  → larger  cross-section dim
    #    Total height  → smaller cross-section dim
    #    Thickness     → smaller_dim / 6  (equal for flange and web by default)
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

    # all_dims[2] = beam axis (longest) — not used for defaults here
    cs_small_val, _ = all_dims[0]  # smaller cross-section dim → total_h default
    cs_large_val, _ = all_dims[1]  # larger  cross-section dim → flange_w default
    thickness_default = cs_small_val / 6.0

    # -------------------------------------------------------------------------
    # 3. Prompt with computed defaults
    # -------------------------------------------------------------------------
    flange_w = ask("Flange width", fmt(cs_large_val))
    if flange_w is None:
        return

    total_h = ask("Total height", fmt(cs_small_val))
    if total_h is None:
        return

    flange_t = ask("Flange thickness", fmt(thickness_default))
    if flange_t is None:
        return

    web_t = ask("Web thickness", fmt(flange_t))  # default = same as flange
    if web_t is None:
        return

    orientation = ui.messageBox(
        "Beam orientation?\n\n"
        "Yes  →  Wide  (flanges along the wider cross-section axis)\n"
        "No   →  Tall  (flanges along the narrower cross-section axis)",
        "Beam Orientation",
        adsk.core.MessageBoxButtonTypes.YesNoButtonType,
        adsk.core.MessageBoxIconTypes.QuestionIconType,
    )
    orient_wide = orientation == adsk.core.DialogResults.DialogYes

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
    # 5. Create beams — cross-section axes come from each body's own OBB
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

            # dims[2] = beam axis (longest)
            beam_len, beam_dir = dims[2]

            # Cross-section axes.
            # Wide: larger dim → flange direction, smaller → height (default).
            # Tall: swap so the beam stands on its narrow face instead.
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
