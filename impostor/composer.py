from pathlib import Path
from typing import Any, Dict, List
import toml
from pydantic import TypeAdapter
from impostor.functions import FunctionType

class Composer:
    def __init__(self, toml_path: str = None):
        self.functions: Dict[str, Any] = {}
        self._execution_plans: Dict[str, List[str]] = {}
        
        if toml_path:
            self.load_from_toml(toml_path)
    
    def load_from_toml(self, toml_path: str):
        """Load functions from TOML file"""
        with open(toml_path, 'r') as f:
            data = toml.load(f)
        
        function_type_adapter = TypeAdapter(FunctionType)
        
        for fn_dict in data["fn"]:
            func = function_type_adapter.validate_python(fn_dict)
            self.functions[func.name] = func
            print(f"Loaded function: {func.name} of type {func.model_kind}")
            print(func)  # Print the function details for debugging
        
        
        # Clear execution plans when loading new functions
        self._execution_plans.clear()
    
    def _build_execution_plan(self, target_func: str) -> List[str]:
        """Build execution order for a target function using DFS + topological sort"""
        dependencies = {}  # func_name -> [dependency_names]
        visited = set()
        
        def collect_dependencies(func_name: str, path: set):
            if func_name in path:
                cycle_str = " -> ".join(list(path) + [func_name])
                raise ValueError(f"Circular dependency detected: {cycle_str}")
            
            if func_name in visited:
                return
            
            visited.add(func_name)
            path.add(func_name)
            
            func = self.functions[func_name]
            func_deps = func.get_dependencies()
            print(f"Collecting dependencies for {func_name}: {func_deps}")
            dependencies[func_name] = list(func_deps)
            
            # Recursively collect dependencies
            for dep in func_deps:
                collect_dependencies(dep, path)
            
            path.remove(func_name)
        
        # Collect all dependencies starting from target
        collect_dependencies(target_func, set())
        
        # Topological sort using Kahn's algorithm
        in_degree = {node: 0 for node in dependencies}
        
        for node in dependencies:
            for dep in dependencies[node]:
                if dep in in_degree:
                    in_degree[node] += 1
        
        queue = [node for node in in_degree if in_degree[node] == 0]
        execution_order = []
        
        while queue:
            current = queue.pop(0)
            execution_order.append(current)
            
            for node in dependencies:
                if current in dependencies[node]:
                    in_degree[node] -= 1
                    if in_degree[node] == 0:
                        queue.append(node)
        
        
        
        if len(execution_order) != len(dependencies):
            raise ValueError("Circular dependency detected in execution plan")
        
        return execution_order
    
    def evaluate(self, function_name: str) -> Any:
        """Evaluate a function and return its result"""
        # Get or build execution plan
        if function_name not in self._execution_plans:
            self._execution_plans[function_name] = self._build_execution_plan(function_name)
        
        execution_order = self._execution_plans[function_name]
        
        # Execute functions in order
        results = {}
        
        for func_name in execution_order:
            func = self.functions[func_name]
            result = func.call_with_dependencies(results)
            results[func_name] = result
        
        return results[function_name]

if __name__ == "__main__":
    composer = Composer("test1.toml")
    result = composer.evaluate("output")
    print(f"Result: {result}")