/// This file contains functions to create a physics sprite out of an svg file.

use base64::prelude::*;
use macroquad::prelude::*;
use rapier2d::prelude::*;
use xml::{attribute::OwnedAttribute, name::OwnedName, reader::{EventReader, XmlEvent}};
use crate::physics::*;

/// Loads a [PhysicsSprite] from an svg with an embedded image covering
/// the canvas representing the appearance, and any paths representing
/// physical shape.
///
/// An easy way to create a shaped physics object is to open an image
/// in inkscape, draw an outline of where the physics should be, and
/// save it as an svg.
///
/// This function will load svg data, make colliders out of any paths
/// it finds, and load the first texture it encounters.
///
/// - `simulation`: The simulation to which this sprite will be added.
/// - `body_builder`: Body properties for this sprite.
/// - `collider_properties`: ColliderBuilder containing properties for this
///   sprite. The shape for this collider is ignored, and can be anything,
///   i.e. `ColliderBuilder.default()`.
pub fn load_svg_physics_sprite(
    simulation: &mut PhysicsSimulation,
    body_builder: &RigidBodyBuilder,
    collider_properties: &ColliderBuilder,
    svg: &str,
    size: Vec2,
    y_down: bool)
-> Option<PhysicsSprite> {
    let canvas_size = load_svg_canvas_size(svg).unwrap_or(size);
    let texture = load_svg_texture(svg)?;
    let mut paths =
        load_physics_paths(svg, size / 2.0, size / canvas_size, 8, y_down);
    // Give the colliders the correct properties
    paths = paths.iter().map(|shaped_collider| {
        let mut result = collider_properties.clone();
        result.shape = shaped_collider.shape.clone();
        result
    }).collect();
    Some(PhysicsSprite {
        body: simulation.create_body(body_builder, &paths),
        texture,
        size
    })
}

/// An easy way to create a shaped physics object is to open an image
/// in inkscape, draw an outline of where the physics should be, and
/// save it as an svg.
///
/// This function will load svg data and make colliders out of any paths
/// it finds.
pub fn load_physics_paths(svg: &str, mut center: Vec2, mut scale: Vec2, curve_segments: i32, y_down: bool) -> Vec<ColliderBuilder> {
    if !y_down {
        scale.y *= -1.0;
        center.y *= -1.0;
    }
    let mut result: Vec<ColliderBuilder> = Vec::new();
    let parser = EventReader::new(svg.as_bytes());
    for e in parser {
        match e {
            Ok(XmlEvent::StartElement { name, attributes, namespace: _ }) => {
                if let Some(path) = load_path(&name, &attributes, center, scale, curve_segments) {
                    result.push(polygon_collider(&path));
                }
            }
            Err(e) => {
                eprintln!("Error: {e}");
                break;
            }
            _ => {}
        }
    }
    result
}

/// Attempts to load a path from an xml tag.
///
/// Returns None if the tag is not a "path", or if path loading fails.
pub fn load_path(name: &OwnedName, attributes: &[OwnedAttribute], center: Vec2, scale: Vec2, curve_segments: i32)
-> Option<Vec<Vec2>> {
    if &name.local_name != "path" {
        return None;
    }
    for attr in attributes {
        if attr.name.local_name.as_str() == "d" {
            let path = parse_path_data(&attr.value, curve_segments)?;
            let polygon: Vec<Vec2> =
                path.iter().map(|x| *x * scale - center).collect();
            return Some(polygon);
        }
    }
    return None;
}

/// Converts the data property of an svg path to a Vec2 list.
///
/// Bezier curves are interpolated to have 8 segments.
/// Arcs are not supported, and are replaced with straight lines.
pub fn parse_path_data(data: &str, curve_segments: i32) -> Option<Vec<Vec2>> {
    let mut tokens = data.split_ascii_whitespace().into_iter();
    let mut previous_control = Option::<Vec2>::None;
    let mut result: Vec<Vec2> = Vec::new();
    let mut mode = "M";
    while let Some(token) = tokens.next() {
        if token.to_lowercase() == "z" {
            break;
        }
        if "mlhvcsqta".contains(&token.to_lowercase()) {
            mode = token;
            continue;
        }
        let lowercase_mode = mode.to_lowercase();
        // Account for lowercase using relative coordinates
        let origin =
            if lowercase_mode == mode {
                *result.last().unwrap_or(&vec2(0.0, 0.0))
            } else {
                vec2(0.0, 0.0)
            };

        match lowercase_mode.as_str() {
            // moveto: Lines must be continuous. Treated as lineto
            "m" => {
                result.push(origin + get_vec(token)?);
            },
            "l" => {
                result.push(origin + get_vec(token)?);
            },
            "h" => {
                result.push(vec2(
                    origin.x + token.parse::<f32>().ok()?,
                    result.last()?.y));
            },
            "v" => {
                result.push(vec2(
                    result.last()?.x,
                    origin.y + token.parse::<f32>().ok()?));
            },
            "c" => {
                previous_control =
                    Some(form_bezier(curve_segments, origin, &mut result, None, 2, token, &mut tokens)?);
            },
            "s" => {
                previous_control =
                    Some(form_bezier(curve_segments, origin, &mut result, previous_control, 2, token, &mut tokens)?);
            },
            "q" => {
                previous_control =
                    Some(form_bezier(curve_segments, origin, &mut result, None, 1, token, &mut tokens)?);
            },
            "t" => {
                previous_control =
                    Some(form_bezier(curve_segments, origin, &mut result, previous_control, 1, token, &mut tokens)?);
            },
            "a" => {
                eprintln!("Arcs are not supported. Replacing with line")
            },
            "z" => { break; },
            _ => {}
        }
    }
    Some(result)
}

pub fn interpolate_bezier(controls: &[Vec2], fraction: f32) -> Vec2 {
    if controls.len() == 0 {
        return vec2(0.0, 0.0);
    }
    if controls.len() == 1 {
        return controls[0];
    }
    if controls.len() == 2 {
        return controls[0] * fraction + controls[1] * (1.0 - fraction);
    }
    return interpolate_bezier(&[
        interpolate_bezier(&controls[0..controls.len()-1], fraction),
        interpolate_bezier(&controls[1..controls.len()], fraction),
    ], fraction)
}

fn get_vec(text: &str) -> Option<Vec2>{
    let coords: Vec<_> = text.split(',').collect();
    Some(vec2(coords.get(0)?.parse().ok()?, coords.get(1)?.parse().ok()?))
}

fn form_bezier(
    bezier_segments: i32,
    origin: Vec2,
    dest: &mut Vec<Vec2>,
    previous_control: Option<Vec2>,
    read_count: usize,
    token: &str,
    tokens: &mut dyn Iterator<Item = &str>)
-> Option<Vec2> {
    let mut controls = vec![*dest.last()?];
    if let Some(previous_control) = previous_control {
        controls.push(*dest.last()? * 2.0 - previous_control)
    }
    controls.push(origin + get_vec(token)?);
    for _ in 0..read_count {
        controls.push(origin + get_vec(tokens.next()?)?);
    }
    let persistent_control = Some(controls[read_count - 2]);
    for i in 0..bezier_segments {
        dest.push(interpolate_bezier(
            &controls,
            (i + 1) as f32 * (1.0 / bezier_segments as f32)));
    }
    persistent_control
}

//////// Extract embedded images ////////

/// An easy way to create a shaped physics object is to open an image
/// in inkscape, draw an outline of where the physics should be, and
/// save it as an svg.
///
/// This function will load svg data and make colliders out of any paths
/// it finds.
pub fn load_svg_texture(svg: &str) -> Option<Texture2D> {
    let parser = EventReader::new(svg.as_bytes());
    for e in parser {
        match e {
            Ok(XmlEvent::StartElement { name, attributes, namespace: _ }) => {
                if let Some(texture) = load_image_tag(&name, &attributes) {
                    return Some(texture);
                }
            }
            Err(e) => {
                eprintln!("Error: {e}");
                break;
            }
            _ => {}
        }
    }
    None
}

pub fn load_image_tag(name: &OwnedName, attributes: &[OwnedAttribute]) -> Option<Texture2D>{
    if &name.local_name != "image" {
        return None
    }
    for attribute in attributes {
        if attribute.name.local_name != "href" {
            continue;
        }
        let image_values: Vec<&str> = attribute.value.split(',').collect();
        if image_values.len() < 2 {
            continue;
        }
        let data = image_values[1].trim()
            .split(' ')
            .filter(|s| !s.is_empty())
            .collect::<Vec<_>>()
            .join("");
        let bytes = match BASE64_STANDARD.decode(&data){
            Err(e) => {eprint!("Error: {}", e); None},
            Ok(val) => Some(val)
        }?;
        return Some(Texture2D::from_file_with_format(&bytes[..], None));
    }
    return None;
}

pub fn load_svg_canvas_size(svg: &str) -> Option<Vec2> {
    let parser = EventReader::new(svg.as_bytes());
    let mut canvas_size: Vec2 = vec2(0.0, 0.0);
    for e in parser {
        match e {
            Ok(XmlEvent::StartElement { name, attributes, namespace: _ }) => {
                if &name.local_name == "svg" {
                    for a in attributes {
                        if a.name.local_name == "width" {
                            canvas_size.x = a.value.parse().ok()?;
                        }
                        if a.name.local_name == "height" {
                            canvas_size.y = a.value.parse().ok()?;
                        }
                    }
                }
            },
            Err(e) => {
                eprintln!("Error: {e}");
                break;
            }
            _ => {}
        }
    }
    if canvas_size.min_element() > 0.0 {
        return Some(canvas_size);
    } else {
        return None;
    }
}