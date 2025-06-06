from typing import Any, Callable, Dict, List, Set, Tuple, Literal, Union
import numpy as np
from dataclasses import dataclass, field, replace
from scipy.interpolate import CubicSpline
from pydantic import BaseModel, Field, TypeAdapter, PrivateAttr # Added PrivateAttr

class Link(BaseModel):
    fn: str

class Function(BaseModel):
    name: str

    def call(self) -> Any:
        """This method should be overridden by subclasses to implement the function's logic."""
        raise NotImplementedError("Subclasses must implement the call method.")
    
    def get_dependencies(self) -> List[str]:
        # Iterate all fields in this instance and filter Link instances
        dependencies = []
        for field_name in self.model_fields:
            field_value = getattr(self, field_name)
            if isinstance(field_value, Link):
                dependencies.append(field_value.fn)
        return dependencies
    
    
    def call_with_dependencies(self, dependencies: Dict[str, Any]) -> Any:
        # Convert self to dict and merge with dependencies
        self_dict = self.model_dump()
        
        # Replace Link objects with actual values from dependencies
        for key, value in self_dict.items():
            if isinstance(getattr(self, key), Link):
                link_fn = getattr(self, key).fn
                if link_fn in dependencies:
                    self_dict[key] = dependencies[link_fn]
        
        # Create a new instance with resolved values
        resolved_instance = self.__class__(**self_dict)
        return resolved_instance.call()

class Curve(Function):
    model_kind: Literal["Curve"] = "Curve" # Discriminator field
    control_points: List[Tuple[float, float]] = Field(default_factory=list)
    _curve: Any = PrivateAttr(default=None) # Declare _curve as a private attribute
    t: float | Link = Field(default=0.0, description="Parameter for evaluating the spline at a specific point")

    def model_post_init(self, __context: Any) -> None:
        """Initialize the spline after the model is created and validated."""
        if len(self.control_points) >= 2:
            self._curve = CubicSpline(*zip(*self.control_points), bc_type="not-a-knot")
        # If control_points are insufficient, self._curve remains None.
        
    def call(self):
        if self._curve is not None:
            return self._curve(self.t)
        # Handle cases where _curve couldn't be initialized (e.g., < 2 control points)
        # or if model_post_init was somehow bypassed (though unlikely with standard Pydantic usage).
        elif len(self.control_points) >= 2: 
            # Fallback: attempt to create the curve on-the-fly if not initialized.
            # This might be desirable if control_points could be mutated after init,
            # though Pydantic models are often treated as immutable.
            temp_curve = CubicSpline(*zip(*self.control_points), bc_type="not-a-knot")
            return temp_curve(self.t)
        return 0.0 # Or raise an error if curve cannot be evaluated

class NormalDistribution(Function):
    model_kind: Literal["NormalDistribution"] = "NormalDistribution" # Discriminator field
    mean: float | Link = Field(default=0.0) 
    std: float

    def call(self) -> float:
        # TODO: Add option to seed the random number generator.
        return np.random.normal(self.mean, self.std)


class Sum(Function):
    model_kind: Literal["Sum"] = "Sum" # Discriminator field
    a: float | Link = Field(default=0.0)
    b: float | Link = Field(default=0.0)
    def call(self) -> float:
        return self.a + self.b
    

class Input(Function):
    model_kind: Literal["Input"] = "Input" # Discriminator field
    value: Any
    
    def call(self) -> Any:
        return self.value
    
class Print(Function):
    model_kind: Literal["Print"] = "Print" # Discriminator field
    message: str | float | Link = Field(default="")

    def call(self) -> None:
        print(f"Output: {str(self.message)}") 
    

    
# List that can hold all possible function types
# Union is implicitly created by the pipe operator in modern Python type hints
FunctionType = Curve | NormalDistribution | Sum | Input | Print

 
if __name__ == "__main__":
    fn = [
        Input(name="time", value=0.5),
        Input(name="mean", value=0.0),
        Curve(name="curve", control_points=[(0, 0), (1, 2), (2, 0), (3, 3)], t=Link(fn="time")),
        NormalDistribution(name="normal_dist", mean=Link(fn="mean"), std=1),
        Sum(name="sum", a=Link(fn="curve"), b=Link(fn="normal_dist")),
        Print(name="output", message=Link(fn="sum")),
    ]
    # print(fn) # Original print

    import toml

    # convert fn to toml
    # .model_dump() will now include the 'model_kind' field automatically
    fn_data = [f.model_dump() for f in fn]
    fn_toml = toml.dumps({"fn": fn_data})
    print("TOML Output:")
    print(fn_toml)

    # load it back
    loaded_data_from_toml = toml.loads(fn_toml)["fn"]
    
    # Create a TypeAdapter for your union type
    function_type_adapter = TypeAdapter(FunctionType) # Or TypeAdapter(List[FunctionType]) for the whole list
    
    fn_loaded = [function_type_adapter.validate_python(f_dict) for f_dict in loaded_data_from_toml]
    
    print("\nLoaded Functions (with correct types):")
    print(fn_loaded)

    # You can verify the types
    # for loaded_f in fn_loaded:
    #     print(f"Name: {loaded_f.name}, Type: {type(loaded_f)}, Kind: {loaded_f.model_kind}")




