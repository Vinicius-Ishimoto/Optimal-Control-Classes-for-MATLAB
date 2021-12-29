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
      Q = cos(X)*sin(Y)-2*X^3;  % equations to be computed
      
To return the variables after all the calculation is done:

- double(Q): return the vectorized variables;

- hderiv: class designed to calculate automatically the jacobian and the hessian of a variable discreted over time. The usual call is:

     V = hderiv(vector,ndiff,location,varargin)
     
     where:
     
          -vector: the vector of values, it creates a sparse value by column;
          -ndiff:  maximum number of variables in the problem;
          -location: the location of the values in 'vector'. The location represents the order of derivative variables.
                     it needs to have the same size as number of rows in 'vector';
     
 Example:
 
      time = [0;1;2;3;4;5;6;7;8;9]; % discrete over 10 values
      x = [0;1;2;3;4;5;6;7;8;9];
      y = [9;8;7;6;5;4;3;2;1;0];
      X = hderiv(x, 10,  1:10);
      Y = hderiv(y, 10, 11:20);
      Q = tan(X)^2-Y/X;  % equations to be computed
      
 In the example, the 10 points over time of "X" are the 1-10 derivatives of the problem, where the "Y" points are the 11-20 derivatives.
