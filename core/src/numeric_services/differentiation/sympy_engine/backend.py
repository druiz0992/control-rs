import sympy as sp
import json
import sys

def extract_symbols(functions, variable_names):
    """
    Extracts the symbols from the function string and variable names.
    """
    # Define the symbolic variables
    variables = sp.symbols(variable_names)
    
    # Parse the function string into a sympy expression
    function = [sp.sympify(f) for f in functions]
    
    return function, variables

import sympy as sp

def compute_gradient(function, variables):
    gradient = []
    with open("/tmp/diff.txt", "a") as log_file:
        #log_file.write(f"Computed gradient Request {function}, {variables}\n")
        # Compute gradients with progress logging
        for i, var in enumerate(variables):
            grad_i = sp.diff(function, var)
            gradient.append(grad_i)
            log_file.write(f"Computed gradient {i+1}/{len(variables)} for variable {var}\n")
            log_file.flush()

        # Convert gradients to strings with progress logging
        gradient_str = []
        #log_file.write(f"Conver gradient request {gradient}\n")
        for j, g in enumerate(gradient):
            gradient_str.append(str(g))
            log_file.write(f"Converted gradient {j+1}/{len(gradient)} to string\n")
            log_file.flush()

    return gradient_str, gradient


def compute_hessian(function, variables, gradient = None):
    if gradient is None:
        # Compute the gradient (partial derivatives)
        gradient = [sp.diff(function[0], var) for var in variables]

    hessian_matrix = [
        [sp.diff(gradient[i], var) for var in variables] for i in range(len(gradient))
    ]
    
    # Convert the symbolic expressions to strings for transfer
    hessian_str = [[str(h) for h in row] for row in hessian_matrix]

    return hessian_str

# Function to parse the function string and compute its gradient and Hessian
def parse_and_compute_derivatives(function, variables, request):
    # Compute the gradient (partial derivatives)
    gradient = [sp.diff(function, var) for var in variables]

    # Compute the Hessian (second-order partial derivatives)
    hessian_matrix = [
        [sp.diff(gradient[i], var) for var in variables] for i in range(len(gradient))
    ]
    
    # Convert the symbolic expressions to strings for transfer
    gradient_str = [str(g) for g in gradient]
    hessian_str = [[str(h) for h in row] for row in hessian_matrix]

    return gradient_str, hessian_str

# Read input (the function description)
input_data = sys.stdin.read()
data = json.loads(input_data)

# Get the function string from Rust
function_list = data['function']
variable_names = data['variables']
request = data['derivatives']

# Compute the gradient and Hessian for the given function
# and variable names
function_exprs, variable_symbols = extract_symbols(function_list, variable_names)

jacobian = []
jacobian_symbols = []
if 'Gradient' in request or 'Jacobian' in request:
    for func_expr in function_exprs:
       gradient, gradient_symbols = compute_gradient(func_expr, variable_symbols)
       jacobian.append(gradient)
       jacobian_symbols.append(gradient_symbols)
else:
    gradient = None
    gradient_symbols = None

if 'Hessian' in request and len(function_exprs) == 1:
    hessian_matrix = compute_hessian(function_exprs, variable_symbols, gradient_symbols)
else:
    hessian_matrix = [[]]

if 'Gradient' not in request or len(function_exprs) > 1:
    gradient = None

if 'Jacobian' not in request:
    jacobian = [[]]

# Return the results as JSON to Rust
output_data = {
    'gradient': gradient,
    'jacobian': jacobian,
    'hessian': hessian_matrix
}

with open('/tmp/hessian.json', 'w') as f:
    json.dump(output_data, f)
print(json.dumps(output_data))
