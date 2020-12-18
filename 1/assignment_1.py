import numpy as np


def generate_edges(phi, n):
    """
    Construct the polyhedral cone with angle phi and n edges.
    :param phi: <float> Cone angle
    :param n: <int> Number of cone edges
    :return: <2-dim np.array> Cone edge matrix of size (n, 3)

    Test cases:
    >>> generate_edges(phi=0.4, n=5)
    array([[ 0.92106099,  0.        ,  0.38941834],
           [ 0.2846235 ,  0.87598106,  0.38941834],
           [-0.745154  ,  0.54138607,  0.38941834],
           [-0.745154  , -0.54138607,  0.38941834],
           [ 0.2846235 , -0.87598106,  0.38941834]])
    >>> generate_edges(phi=np.pi/4, n=3)
    array([[ 0.70710678,  0.        ,  0.70710678],
           [-0.35355339,  0.61237244,  0.70710678],
           [-0.35355339, -0.61237244,  0.70710678]])
    """
    # ------------------------------------------------
    # FILL WITH YOUR CODE
    v = np.zeros(3)
    V = np.zeros((n,3))
    theta = 0

    for i in range(n):
        theta = 2 * np.pi * i / n 
        v = np.array([np.cos(theta)*np.cos(phi), np.sin(theta)*np.cos(phi), np.sin(phi)])
        V[i:] = v

    cone_edges = V # TODO: Replace None with your result
    # ------------------------------------------------
    return cone_edges


def compute_normals(cone_edges):
    """
    Compute the facet normals given the cone edge matrix.
    :param cone_edges: <2-dim np.array> Cone edge matrix of size (n, 3)
    :return: <2-dim np.array> Facet normals matrix of size (n, 3)

    Test cases:
    >>> compute_normals(np.array([[ 0.70710678,  0.        ,  0.70710678],[-0.35355339,  0.61237244,  0.70710678], [-0.35355339, -0.61237244,  0.70710678]]))
    array([[-0.4330127 , -0.75      ,  0.4330127 ],
           [ 0.86602541,  0.        ,  0.4330127 ],
           [-0.4330127 ,  0.75      ,  0.4330127 ]])
    """
    # ------------------------------------------------
    # FILL WITH YOUR CODE
    V = cone_edges 
    # print(V)
    n = len(V)
    # print(n)
    s = np.zeros(3)
    S = np.zeros((n,3)) 
    # print(s,"\n",S)

    for i in range(n):
        if i >= n-1:
            s = np.cross(V[i],V[0])
        else:
            s = (np.cross(V[i],V[(i+1)]))
        S[i:] = s

    facet_normals = S  # TODO: Replace None with your result
    # ------------------------------------------------
    return facet_normals


def compute_minimum_distance_from_facet_normals(a, facet_normals):
    """
    Compute the minimum distance from a point 'a' to the polyhedral
    cone parametrized by the given facet normals.
    :param a: <np.array> 3D point
    :param facet_normals: <2-dim np.array> Facet normals matrix of size (n, 3)
    :return: <float> Minimum distance from 'a' to the cone.

    Test cases:
    >>> compute_minimum_distance_from_facet_normals(a=np.array([0,0,1]), facet_normals=np.array([[-4.33012702e-01, -7.50000000e-01,  4.33012702e-01], [ 8.66025404e-01, -3.33066907e-16,  4.33012702e-01], [-4.33012702e-01,  7.50000000e-01,  4.33012702e-01]]))
    0.4472135954999579
    >>> compute_minimum_distance_from_facet_normals(a=np.array([10,-10,0.2]), facet_normals=np.array([[-4.33012702e-01, -7.50000000e-01,  4.33012702e-01], [ 8.66025404e-01, -3.33066907e-16,  4.33012702e-01], [-4.33012702e-01,  7.50000000e-01,  4.33012702e-01]]))
    3.3632734550894954
    """
    # ------------------------------------------------
    # FILL WITH YOUR CODE
    S = facet_normals
    d = np.zeros(len(S))
    # print(d)
    
    for i in range(len(S)):
        # print(S[i])
        S_abs = np.sqrt(S[i][0]**2+S[i][1]**2+S[i][2]**2)
        # d[i] = np.absolute(np.dot(S[i],a) / S_abs)
        d[i] = np.dot(S[i],a) / S_abs
        # print(np.dot(S[i],a))
        # print(d[i])
    d_star = np.amin(np.absolute(d))

    minimum_distance = d_star  # TODO: Replace None with your result
    # ------------------------------------------------
    return minimum_distance


def compute_minimum_distance(a, n, phi):
    """
    Compute the minimum distance from a point 'a' to the polyhedral
    cone of n edges and angle phi
    :param a: <np.array> 3D point
    :param n: <int> Number of cone edges
    :param phi: <float> Cone angle
    :return: <float> Minimum distance from 'a' to the cone.

    Test cases:
    >>> compute_minimum_distance(a=np.array([0.2,-0.3,0.1]), n=7, phi=0.3)
    0.0062024061079438446
    >>> compute_minimum_distance(a=np.array([0.2,-0.3,0.1]), n=10, phi=0.01)
    0.0962065349510294
    """
    # ------------------------------------------------
    # FILL WITH YOUR CODE

    V = generate_edges(phi,n)
    S = compute_normals(V)
    d_star = compute_minimum_distance_from_facet_normals(a,S)

    minimum_distance = d_star  # TODO: Replace None with your result
    # ------------------------------------------------
    return minimum_distance


def check_is_interior_point(a, n, phi):
    """
    Return whether a is an interior point of the polyhedral cone
    of n edges and angle phi
    :param a: <np.array> 3D point
    :param n: <int> Number of cone edges
    :param phi: <float> Cone angle
    :return: <bool> If a is an interior point

    Test cases:
    >>> check_is_interior_point(a=np.array([0.2,-0.3,0.1]), n=7, phi=0.3)
    False
    >>> check_is_interior_point(a=np.array([0.2,-0.3,10]), n=7, phi=0.3)
    True
    """
    # ------------------------------------------------
    # FILL WITH YOUR CODE

    V = generate_edges(phi,n)
    S = compute_normals(V)
    d = np.zeros(len(S))
    # print(d)
    
    for i in range(len(S)):
        # print(S[i])
        S_abs = np.sqrt(S[i][0]**2+S[i][1]**2+S[i][2]**2)
        # d[i] = np.absolute(np.dot(S[i],a) / S_abs)
        d[i] = np.dot(S[i],a) / S_abs
        # print(np.dot(S[i],a))
        # print(d[i])
    d_star = np.amin(d)

    result = d_star > 0

    is_interior_point = result  # TODO: Replace None with your result
    # ------------------------------------------------
    return is_interior_point


if __name__ == "__main__":
    # You can use this main function to test your code with some test values

    # Test values
    phi = 30. * np.pi / 180.
    n = 4
    a = np.array([0.00, 0.01, 1.00])

    # Example for testing your functions
    cone_edges = generate_edges(phi, n)
    print("Generate Edges")
    print(cone_edges)
    print()
    facet_normals = compute_normals(cone_edges)
    print("Comptue Normals")
    print(facet_normals)
    print()
    minimum_distance = compute_minimum_distance_from_facet_normals(a, facet_normals)
    print("Compute Minimum Distance from Facet Normals")
    print(minimum_distance)
    print()
    minimum_distance_ = compute_minimum_distance(a, n, phi)
    print("Compute Minimum Distance")
    print(minimum_distance_)
    print()
    Result = check_is_interior_point(a, n, phi)
    print("Check is Interior Point")
    print(Result)
    print()

    # Automated Testing:
    # import doctest

    # Run tests cases for all functions:
    # doctest.testmod(verbose=True) # Uncomment to test all functions

    # Tests for only a desired function (uncomment the one for the function to test):
    # doctest.run_docstring_examples(generate_edges, globals(), verbose=True)   # Uncomment to test generate_edges
    # doctest.run_docstring_examples(compute_normals, globals(), verbose=True)  # Uncomment to test compute_normals
    # doctest.run_docstring_examples(compute_minimum_distance_from_facet_normals, globals(), verbose=True)  # Uncomment to test compute_minimum_distance_from_facet_normals
    # doctest.run_docstring_examples(compute_minimum_distance, globals(), verbose=True) # Uncomment to test compute_minimum_distance
    # doctest.run_docstring_examples(check_is_interior_point, globals(), verbose=True)  # Uncomment to test check_is_interior_point

