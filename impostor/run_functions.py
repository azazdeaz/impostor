from typing import Any, Callable, Dict, List, Tuple
import numpy as np
from dataclasses import dataclass, field, replace
from scipy.interpolate import CubicSpline

class Curve:
    def __init__(self, control_points: list[tuple[float, float]]):
        self._control_points = control_points
        self.curve = CubicSpline(*zip(*self._control_points), bc_type="not-a-knot")
        

    def __call__(self, t):
        return self.curve(t)
    
@dataclass
class NormalDistribution:
    mean: float
    std: float

    def __call__(self):
        # TODO: Add option to seed the random number generator.
        return np.random.normal(self.mean, self.std)

@dataclass
class Sum:
    def __call__(self, *args: Any) -> float:
        return sum(args)
    
@dataclass
class Input:
    value: Any
    
    def __call__(self):
        return self.value

@dataclass   
class Function:
    name: str
    callable: Callable
    inputs: List[Tuple[str, str]]
    outputs: List[str]

@dataclass
class Composer:
    functions: Dict[str, Function] = field(default_factory=dict)
    _execution_plans: Dict[Tuple[str, str], List[Tuple[str, str]]] = field(default_factory=dict, init=False)

    def add_function(self, func: Function):
        if func.name in self.functions:
            raise ValueError(f"Function with name {func.name} already exists.")
        self.functions[func.name] = func
        # Invalidate all cached execution plans when graph changes
        self._execution_plans.clear()
    
    def _build_execution_plan(self, target_func: str, target_output: str) -> List[Tuple[str, str]]:
        """Build and cache the execution order for a target output using DFS + topological sort"""
        
        # First, collect all dependencies using DFS
        dependencies = {}  # (func, output) -> [(input_func, input_output), ...]
        visited = set()
        
        def collect_dependencies(func_name: str, output_name: str, path: set):
            key = (func_name, output_name)
            
            # Cycle detection
            if key in path:
                cycle_path = list(path) + [key]
                cycle_str = " -> ".join(f"{f}.{o}" for f, o in cycle_path)
                raise ValueError(f"Circular dependency detected: {cycle_str}")
            
            if key in visited:
                return
            
            visited.add(key)
            path.add(key)
            
            func = self.functions[func_name]
            dependencies[key] = func.inputs.copy()
            
            # Recursively collect dependencies
            for input_func, input_output in func.inputs:
                collect_dependencies(input_func, input_output, path)
            
            path.remove(key)
        
        # Collect all dependencies starting from target
        collect_dependencies(target_func, target_output, set())
        
        # Now perform topological sort to get execution order
        in_degree = {node: 0 for node in dependencies}
        
        # Calculate in-degrees
        for node in dependencies:
            for dep in dependencies[node]:
                if dep in in_degree:  # Only count dependencies that are in our subgraph
                    in_degree[node] += 1
        
        # Kahn's algorithm for topological sort
        queue = [node for node in in_degree if in_degree[node] == 0]
        execution_order = []
        
        while queue:
            current = queue.pop(0)
            execution_order.append(current)
            
            # Update in-degrees of dependent nodes
            for node in dependencies:
                if current in dependencies[node]:
                    in_degree[node] -= 1
                    if in_degree[node] == 0:
                        queue.append(node)
        
        # Verify we processed all nodes (no cycles)
        if len(execution_order) != len(dependencies):
            raise ValueError("Circular dependency detected in execution plan")
        
        return execution_order
    
    def evaluate(self, function_name: str, output_name: str) -> Any:
        key = (function_name, output_name)
        
        # Get or build execution plan
        if key not in self._execution_plans:
            self._execution_plans[key] = self._build_execution_plan(function_name, output_name)
        
        execution_order = self._execution_plans[key]
        
        # Execute functions in order (no memoization, fresh computation each time)
        results = {}
        
        for func_name, out_name in execution_order:
            func = self.functions[func_name]
            
            # Collect input values
            input_values = []
            for input_func, input_output in func.inputs:
                input_values.append(results[(input_func, input_output)])
            
            # Execute function
            result = func.callable(*input_values)
            results[(func_name, out_name)] = result
        
        return results[(function_name, output_name)]
    
    def invalidate_cache(self):
        """Manually invalidate execution plan cache if needed"""
        self._execution_plans.clear()
    
    def get_execution_plan(self, function_name: str, output_name: str) -> List[Tuple[str, str]]:
        """Get the execution plan for debugging/inspection"""
        key = (function_name, output_name)
        if key not in self._execution_plans:
            self._execution_plans[key] = self._build_execution_plan(function_name, output_name)
        return self._execution_plans[key].copy()
    

if __name__ == "__main__":
    # Example usage of the Curve class
    control_points = [(0, 0), (1, 2), (2, 0), (3, 3)]
    curve = Curve(control_points)
    
    # Evaluate the curve at t=1.5
    point = curve(1.5)
    print(f"Point on the curve at t=1.5: {point}")
    

    t = Input(value=1.5)

    composer = Composer()
    composer.add_function(Function(
        name="t",
        callable=t,
        inputs=[],
        outputs=["out"]
    ))

    composer.add_function(Function(
        name="curve1",
        callable=curve,
        inputs=[("t", "out")],
        outputs=["out"]
    ))
    for _t in np.linspace(0, 3, 10):
        t.value = _t
        point = composer.evaluate("curve1", "out")
        print(f"Point on the curve at t={_t}: {point}")






