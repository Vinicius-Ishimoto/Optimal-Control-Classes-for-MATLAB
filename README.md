# Optimal-Control-Classes-for-MATLAB
Optimal control classes for differentiation in MATLAB

# Classes description
The two main classes uses automatic differentiation to calculate the Jacobian and Hessian automatically, without the need for external files or writing a specific function for use. They are specially designed for optimal control, where each variable is a vector discretized over time. Some example of uses are are shown bellow:

- vect: class designed to do calculations of a vector as a single variable (convert a vector to a singular variable). The call is:

     V = vect(X, ndiff)
     
     where:
     
          V: Final vector;
          X: vector or matrix of variables.
          The function recognizes each column as a variable with each row considered as a point of the variable over time
          ndiff: number of points discrete over time. Its only defined when inputing a matrix of variables.
 
 Example:
 
      time = [0;1;2;3;4;5;6;7;8;9];
      x = [0;1;2;3;4;5;6;7;8;9];
      y = [9;8;7;6;5;4;3;2;1;0];
      X = vect(x);
      Y = vect(y);
      Q = cos(X)*sin(Y)-2*X^3;
      
To return the variables after all the calculation is done:

- double(Q): return the vectorized variables;

