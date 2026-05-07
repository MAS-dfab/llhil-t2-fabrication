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
    # Get the start and end points of both beams
    a_start = Point(*element_a.start)
    a_end = Point(*element_a.end)
    b_start = Point(*element_b.start)
    b_end = Point(*element_b.end)

    # Create vectors for both beams
    a_vector = Vector.from_start_end(a_start, a_end)
    b_vector = Vector.from_start_end(b_start, b_end)

    # Calculate the angle between the two vectors
    angle = angle_vectors(a_vector, b_vector)

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
    start = Point(*beam.start)
    end = Point(*beam.end)

    # Create a vector from the start to the end point
    beam_vector = Vector.from_start_end(start, end)

    # Calculate the angle between the beam vector and the reference frame's x-axis
    angle = angle_vectors(beam_vector, reference_frame.xaxis)

    # If the angle is greater than 90 degrees, reverse the beam direction
    if abs(angle) > 90:
        return Beam(start=beam.end, end=beam.start, **beam.attributes)
    
    return beam

