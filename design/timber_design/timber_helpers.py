"""Beam classification utilities for compas_timber models."""

from compas.geometry import Point, Line, Vector, Frame, angle_vectors
from compas_timber.elements import Beam

def is_planar(element_a, element_b):
    """Check if two beams are planar.

    Parameters
    ----------
    element_a : :class:`compas_timber.elements.Beam`
        The first beam.
    element_b : :class:`compas_timber.elements.Beam`
        The second beam.

    Returns
    -------
    bool
        ``True`` if the beams are planar, ``False`` otherwise.

    """
    # Get frame from each beam
    frame_a = element_a.frame
    frame_b = element_b.frame

    # Match orientation of the frames
    frame_a.xaxis = Vector(0, 0, 1)
    frame_b.xaxis = Vector(0, 0, 1)

    # Calculate the angle between the two vectors
    angle = angle_vectors(frame_a.normal, frame_b.normal, deg=True)

    # If the angle is close to 0 or 180 degrees, the beams are planar
    return abs(angle) < 1e-6 or abs(angle - 180) < 1e-6

def orient_beam(beam, reference_frame):
    """Orient a beam according to a reference frame.

    Parameters
    ----------
    beam : :class:`compas_timber.elements.Beam`
        The beam to orient.
    reference_frame : :class:`compas.geometry.Frame`
        The reference frame for orientation.

    Returns
    -------
    :class:`compas_timber.elements.Beam`
        The oriented beam.

    """
    # Get the start and end points of the beam
    start = Point(*beam.centerline.start)
    end = Point(*beam.centerline.end)

    # Create a vector from the start to the end point
    beam_vector = Vector.from_start_end(start, end)

    # Calculate the angle between the beam vector and the reference frame's x-axis
    angle = angle_vectors(beam_vector, reference_frame.xaxis)

    # If the angle is greater than 90 degrees, reverse the beam direction
    if abs(angle) > 90:
        return Beam(start=beam.centerline.end, end=beam.centerline.start, **beam.attributes)

    return beam

def orient_polyline_to_world_z(polyline):
    """Orient a polyline to align with the world Z-axis.

    Parameters
    ----------
    polyline : list of :class:`compas.geometry.Point`
        The polyline to orient.

    Returns
    -------
    list of :class:`compas.geometry.Point`
        The oriented polyline.
    
    frame : :class:`compas.geometry.Frame`
        The local frame of the polyline, with the Z-axis aligned to the world Z-axis

    """
    # Calculate the normal vector of the polyline
    if len(polyline) < 3:
        raise ValueError("Polyline must have at least 3 points to calculate a normal vector.")
    
    v1 = Vector.from_start_end(polyline[0], polyline[1])
    v2 = Vector.from_start_end(polyline[1], polyline[2])
    normal = v1.cross(v2)

    # Calculate the angle between the normal vector and the world Z-axis
    world_z = Vector(0, 0, 1)
    angle = angle_vectors(normal, world_z)

    # If the angle is greater than 90 degrees, reverse the order of the points
    if abs(angle) > 90:
        return list(reversed(polyline))

    # Create the local frame for the polyline
    local_z_frame = Frame.from_points(polyline[0], polyline[1], polyline[2])

    return polyline, local_z_frame