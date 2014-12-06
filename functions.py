'''
functions.py
Written 2008-6-4 by Peter Mawhorter
This module defines a variety of funtions useful for the FastSLAM
algorithm. Some of them work with objects from the components module.
Many of these functions work with 2x2 matrices. For these functions, a
2x2 matrix is a 2-tuple (or length-2 list) containing two 2-tuples (or
length-2 lists) of numbers. The outer tuple is a list of rows, each of
the inner tuples is a row of (2) numbers (row-major order).
'''

import math

class SingularityError(Exception):
  '''
  This error indicates that a matrix is singular, and thus cannot be
  inverted.
  '''
  def __init__(self, matrix):
    '''
    This constructor takes as an argument the singular matrix.
    '''
    self.matrix = matrix

  def __str__(self):
    return "Singular Matrix: " + str(self.matrix)

  def __repr__(self):
    return "SingularityError(" + repr(self.matrix) + ")"

def rTheta(x, y):
  '''
  This function takes a pair of Cartesian coordinates x and y and
  returns an equivalent pair of radial coordinates, r and theta.
  Following standard conventions, theta is measured from the positive
  x- axis counter-clockwise, and will always have a value between -pi
  and pi.
  '''
  theta = math.atan2(y,x)
  r = (x**2 + y**2)**0.5
  return r, theta

def xY(r, theta):
  '''
  This function takes a pair of radial coordinates r and theta and
  returns an equivalent pair of Cartesian coordinates, x and y. It
  accepts any value of theta, even values greater than 2*pi, but it
  treats theta as an angle measured from the positive x-axis in the
  counter-clockwise direction.
  '''
  x = r*math.cos(theta)
  y = r*math.sin(theta)
  return x, y

def bearingRangeJacobian(x, y):
  '''
  Short version: takes a pair of numbers x and y and returns the
  Jacobian of the function mapping Cartesian to radial coordinates at
  that point as a 2-dimensional 2x2 tuple of numbers in row-major order
  (the outer list is a list of rows, and the inner lists are each a
  single row).

  Long version:

  This function calculates the Jacobian of the function that maps a
  position to a range and a bearing. That is, The function that takes
  values x and y and returns values r and theta, or the function that
  maps from Cartesian to radial coordinates. The Jacobian is just the
  2x2 matrix of partial derivatives with respect to the function's input
  coordinate system. As a 2x2 matrix it holds four values: the partial
  derivative of the range with respect to x, the partial derivative of
  the range with respect to y, and the partials of the bearing with
  respect to x and y. Graphically:

      [                    ]
      |    dr        dr    |
      |    --        --    |
      |    dx        dy    |
      |                    |
      |                    |
      |  dtheta    dtheta  |
      |  ------    ------  |
      |    dx        dy    ]
      [                    ]

  As a matrix, the Jacobian describes how the bearing and range would
  vary if the x and y coordinates changed a bit (just like a normal
  derivative). The input to this function, a pair of numbers x and y,
  describes where to evaluate the Jacobian (its value is different at
  different points in the Cartesian plane). The values of this Jacobian
  at arbitrary x and y are:

      [                          ]
      |   cos(th)      sin(th)   |
      |                          |
      |  -sin(th)/r   cos(th)/r  |
      [                          ]

  Where th is the bearing theta to the point (x,y) and r is the range to
  the point (x,y). Another way to look at the Jacobian is as a transform
  operator from uncertainties in Cartesian coordinates to uncertainties
  in radial coordinates. That is, multiplying a matrix of covariances in
  Cartesian space by this Jacobian will result in a matrix of
  covariances in radial space. The inverse Jacobian can likewise be used
  to transform a covariance back from radial to Cartesian coordinates.
  '''
  r = (x**2 + y**2)**0.5 # The bearing to the point in the plane.
  r2 = r**2 # Speed things up a tiny bit by not computing this twice.

  if r == 0:
    # TODO: Fix this.
    raise ValueError('')

  # Compute the Jacobian and return it. Since r is the hypotenuse of the
  # right triangle with sides x, y, and r (and angle theta between sides
  # x and r), cos(theta) = x/r and sin(theta) = y/r, which is quite
  # convenient:
  return ((  x/r  ,  y/r  ),
          ( -y/r2 ,  x/r2 ))

def inverse(((a, b),
            (c, d))):
  '''
  This method takes a 2x2 matrix as a two-dimensional tuple (or list) in
  row-major order (the outer list is a list of rows, and the inner lists
  are each a single row)  and returns a new two-dimensional 2x2 tuple
  that is the inverse of the entered matrix. If the matrix given is
  singular (that it, does not have an inverse), a SingularityError is
  returned.
  '''
  divisor = float(a*d - b*c)
  if divisor == 0:
    raise SingularityError(((a, b), (c, d)))
  return ((d/divisor, -b/divisor), (-c/divisor, a/divisor))

def transpose(((a, b),
               (c, d))):
  '''
  This method takes a 2x2 matrix as a two-dimensional tuple (or list) in
  row-major order (the outer list is a list of rows, and the inner lists
  are each a single row) and returns a new two-dimensional 2x2 tuple
  that is the transpose of the entered matrix.
  '''
  return ((a, c),
          (b, d))

def determinant(((a, b),
                 (c, d))):
  '''
  This method takes a 2x2 matrix as a two-dimensional tuple (or list) in
  row-major order. It returns a single number, the determinant of the
  matrix.
  '''
  return a*d - b*c

def add(((a, b),
         (c, d)),
        ((e, f),
         (g, h))):
  '''
  This method takes two 2x2 matrices as two-dimensional tuples (or
  lists) in row-major order. It returns a new matrix that is the first
  matrix plus the second.
  '''
  return ((  a + e,   b + f  ),
          (  c + g,   d + h  ))

def multiply(((a, b),
              (c, d)),
             ((e, f),
              (g, h))):
  '''
  This method takes two 2x2 matrices as two-dimensional tuples (or
  lists) in row-major order. It returns a new matrix that is the matrix
  product of the input matrices (it multiplies the first matrix by the
  second on the first's left, not vice-versa).
  '''
  return ((  a*e + b*g,   a*f + b*h  ),
          (  c*e + d*g,   c*f + d*h  ))

def multVector(((a, b),
                (c, d)),
               (e, f)):
  '''
  This method takes a 2x2 matrix as a two-dimensional tuple (or list) in
  row-major order and a vector (interpreted as a column vector, as
  required for the multiplication to work) of 2 elements and multiplies
  the matrix by the vector on the right, returning a 2-element tuple
  that contains the resulting column vector.
  '''
  return (a*e + b*f,
          c*e + d*f)

def multScalar(((a, b),
                (c, d)),
               scalar):
  '''
  This method takes a 2x2 matrix as a two-dimensional tuple (or list) in
  row-major order and a scalar and multiplies the matrix by the scalar,
  returning the resulting matrix as a 2x2 two-dimensional tuple in
  row-major order.
  '''
  return ((a*scalar, b*scalar),
          (c*scalar, d*scalar))

def ABAtranspose(A, B):
  '''
  This convenience function returns the result of multiplying A by B and
  then multiplying the result by A transpose. A and B should both be
  matrices, or two-dimensional 2x2 tuples or lists in row-major order.
  '''
  return multiply(multiply(A, B), transpose(A))
