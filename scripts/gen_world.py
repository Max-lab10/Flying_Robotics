import random
import math
import xml.etree.ElementTree as ET
import os

def generate_bent_pipeline():
    start = (1.0, 1.0)
    total_length = random.uniform(5.0, 10.0)
    bend_ratio = random.uniform(0.3, 0.7)
    len1 = total_length * bend_ratio
    len2 = total_length - len1

    angle1 = 0.0
    bend_angle_deg = random.uniform(-30.0, 30.0)
    bend_angle = math.radians(bend_angle_deg)
    angle2 = angle1 + bend_angle

    bend_point = (
        start[0] + len1 * math.cos(angle1),
        start[1] + len1 * math.sin(angle1)
    )
    end = (
        bend_point[0] + len2 * math.cos(angle2),
        bend_point[1] + len2 * math.sin(angle2)
    )

    segments = [
        {"start": start, "end": bend_point, "length": len1, "angle": angle1},
        {"start": bend_point, "end": end, "length": len2, "angle": angle2}
    ]

    total_len = len1 + len2
    taps = []
    min_dist = 0.75
    attempts = 0

    while len(taps) < 5 and attempts < 100:
        attempts += 1
        s = random.uniform(0.1 * total_len, 0.9 * total_len)
        if s <= len1:
            seg = segments[0]
            t = (s - 0.0) / seg["length"]
            x = seg["start"][0] + t * (seg["end"][0] - seg["start"][0])
            y = seg["start"][1] + t * (seg["end"][1] - seg["start"][1])
            direction_angle = seg["angle"]
        else:
            seg = segments[1]
            t = (s - len1) / seg["length"]
            x = seg["start"][0] + t * (seg["end"][0] - seg["start"][0])
            y = seg["start"][1] + t * (seg["end"][1] - seg["start"][1])
            direction_angle = seg["angle"]

        normal_angle = direction_angle + random.choice([math.pi/2, -math.pi/2])
        tap_length = random.uniform(1.0, 2.0)

        tap_end_x = x + tap_length * math.cos(normal_angle)
        tap_end_y = y + tap_length * math.sin(normal_angle)

        too_close = False
        for tx, ty, *_ in taps:
            if math.hypot(x - tx, y - ty) < min_dist:
                too_close = True
                break

        if not too_close:
            taps.append((x, y, tap_end_x, tap_end_y, tap_length))

    return {
        "main_segments": segments,
        "main_radius": 0.1,
        "taps": taps
    }


def ensure_ground_plane_exists(world_element):
    includes = world_element.findall("include")
    ground_plane_exists = False
    for include in includes:
        uri = include.find("uri")
        if uri is not None and "ground_plane" in uri.text:
            ground_plane_exists = True
            break

    if not ground_plane_exists:
        ground_include = ET.SubElement(world_element, "include")
        ground_uri = ET.SubElement(ground_include, "uri")
        ground_uri.text = "model://ground_plane"
        ground_pose = ET.SubElement(ground_include, "pose")
        ground_pose.text = "0 0 -0.01 0 0 0"
        print("Добавлен model://ground_plane")

    return ground_plane_exists


def ensure_sun_exists(world_element):
    includes = world_element.findall("include")
    sun_exists = False
    for include in includes:
        uri = include.find("uri")
        if uri is not None and "sun" in uri.text:
            sun_exists = True
            break

    if not sun_exists:
        sun_include = ET.SubElement(world_element, "include")
        sun_uri = ET.SubElement(sun_include, "uri")
        sun_uri.text = "model://sun"
        print("Добавлен model://sun")

    return sun_exists


def add_pipeline_to_existing_world(template_path, output_path, pipeline_data):
    tree = ET.parse(template_path)
    root = tree.getroot()

    world = root.find("world")
    if world is None:
        world = root

    ensure_ground_plane_exists(world)
    ensure_sun_exists(world)

    for i, seg in enumerate(pipeline_data["main_segments"]):
        x0, y0 = seg["start"]
        x1, y1 = seg["end"]
        length = seg["length"]
        angle = seg["angle"]

        pipe = ET.SubElement(world, "model", name=f"main_pipe_{i}")
        static = ET.SubElement(pipe, "static")
        static.text = "true"  

        pose = ET.SubElement(pipe, "pose")
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        pose.text = f"{cx:.3f} {cy:.3f} 0.1 0 -1.5708 {angle:.3f}"

        link = ET.SubElement(pipe, "link", name="link")
        visual = ET.SubElement(link, "visual", name="visual")
        geom = ET.SubElement(visual, "geometry")
        cyl = ET.SubElement(geom, "cylinder")
        ET.SubElement(cyl, "radius").text = str(pipeline_data["main_radius"])
        ET.SubElement(cyl, "length").text = f"{length:.3f}"

    for i, (x0, y0, x1, y1, length) in enumerate(pipeline_data["taps"]):
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        angle = math.atan2(y1 - y0, x1 - x0)

        tap = ET.SubElement(world, "model", name=f"tap_{i}")
        static = ET.SubElement(tap, "static")
        static.text = "true"  

        pose = ET.SubElement(tap, "pose")
        pose.text = f"{cx:.3f} {cy:.3f} 0.05 0 -1.5708 {angle:.3f}"

        link = ET.SubElement(tap, "link", name="link")
        visual = ET.SubElement(link, "visual", name="visual")
        geom = ET.SubElement(visual, "geometry")
        cyl = ET.SubElement(geom, "cylinder")
        ET.SubElement(cyl, "radius").text = "0.05"
        ET.SubElement(cyl, "length").text = f"{length:.3f}"

    scene = world.find("scene")
    if scene is None:
        scene = ET.SubElement(world, "scene")
        ambient = ET.SubElement(scene, "ambient")
        ambient.text = "0.8 0.8 0.8 1"
        background = ET.SubElement(scene, "background")
        background.text = "0.8 0.9 1 1"
        shadows = ET.SubElement(scene, "shadows")
        shadows.text = "false"
        grid = ET.SubElement(scene, "grid")
        grid.text = "false"

    includes = world.findall("include")
    aruco_replaced = False
    for incl in includes:
        uri_elem = incl.find("uri")
        if uri_elem is not None and uri_elem.text.strip() == "model://aruco_map_txt":
            uri_elem.text = "model://aruco_cmit_txt"
            aruco_replaced = True
            break


    tree.write(output_path, encoding="utf-8", xml_declaration=True)


def create_world_from_scratch(output_path, pipeline_data):
    sdf = ET.Element("sdf", version="1.6")
    world = ET.SubElement(sdf, "world", name="default")

    for model in ["sun", "ground_plane"]:
        incl = ET.SubElement(world, "include")
        uri = ET.SubElement(incl, "uri")
        uri.text = f"model://{model}"
        if model == "ground_plane":
            pose = ET.SubElement(incl, "pose")
            pose.text = "0 0 -0.01 0 0 0"

    scene = ET.SubElement(world, "scene")
    ET.SubElement(scene, "ambient").text = "0.8 0.8 0.8 1"
    ET.SubElement(scene, "background").text = "0.8 0.9 1 1"
    ET.SubElement(scene, "shadows").text = "false"
    ET.SubElement(scene, "grid").text = "false"

    physics = ET.SubElement(world, "physics", name="default_physics", default="0", type="ode")
    ET.SubElement(physics, "gravity").text = "0 0 -9.8066"
    ode = ET.SubElement(physics, "ode")
    solver = ET.SubElement(ode, "solver")
    ET.SubElement(solver, "type").text = "quick"
    ET.SubElement(solver, "iters").text = "10"
    ET.SubElement(solver, "sor").text = "1.3"


    for i, seg in enumerate(pipeline_data["main_segments"]):
        x0, y0 = seg["start"]
        x1, y1 = seg["end"]
        length = seg["length"]
        angle = seg["angle"]

        pipe = ET.SubElement(world, "model", name=f"main_pipe_{i}")
        ET.SubElement(pipe, "static").text = "true"
        pose = ET.SubElement(pipe, "pose")
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        pose.text = f"{cx:.3f} {cy:.3f} 0.1 0 -1.508 {angle:.3f}"

        link = ET.SubElement(pipe, "link", name="link")
        visual = ET.SubElement(link, "visual", name="visual")
        geom = ET.SubElement(visual, "geometry")
        cyl = ET.SubElement(geom, "cylinder")
        ET.SubElement(cyl, "radius").text = str(pipeline_data["main_radius"])
        ET.SubElement(cyl, "length").text = f"{length:.3f}"

    for i, (x0, y0, x1, y1, length) in enumerate(pipeline_data["taps"]):
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        angle = math.atan2(y1 - y0, x1 - x0)

        tap = ET.SubElement(world, "model", name=f"tap_{i}")
        ET.SubElement(tap, "static").text = "true"
        pose = ET.SubElement(tap, "pose")
        pose.text = f"{cx:.3f} {cy:.3f} 0.05 0 -1.5708 {angle:.3f}"

        link = ET.SubElement(tap, "link", name="link")
        visual = ET.SubElement(link, "visual", name="visual")
        geom = ET.SubElement(visual, "geometry")
        cyl = ET.SubElement(geom, "cylinder")
        ET.SubElement(cyl, "radius").text = "0.1"
        ET.SubElement(cyl, "length").text = f"{length:.3f}"

    tree = ET.ElementTree(sdf)
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return output_path


if __name__ == "__main__":
    template_world = "/home/clover/catkin_ws/src/clover/clover_simulation/resources/worlds/clover_aruco.world"
    output_world = "oil_pipeline.world"

    pipeline = generate_bent_pipeline()
    add_pipeline_to_existing_world(template_path=template_world, output_path=output_world, pipeline_data=pipeline)
