from typing import List
from . import l_systems as ls
from .l_systems import Symbol, Rule, F, Yaw, Pitch, Roll, T, BranchOpen, BranchClose
from .curve import BezierCurve2D

class Leaf(Symbol):
    def __str__(self) -> str:
        return "Leaf"
    

def create_leaf() -> List[Symbol]:
    midrib_division = 12
    sec_vein_division = 6
    
    leaf: List[Symbol] = [Leaf()]

    for mr in range(midrib_division):
        # Create all symbols explicitly to help the type checker
        symbols: List[Symbol] = []

        length = 0 if mr == 0 else 0.5
        symbols.append(F(length=length))
        symbols.append(BranchOpen())
        symbols.append(Yaw(angle=-75))
        
        # Add secondary vein on left side
        for _ in range(sec_vein_division):
            symbols.append(F(length=0.5))
        
        symbols.append(BranchClose())
        symbols.append(BranchOpen())
        symbols.append(Yaw(angle=75))
        
        # Add secondary vein on right side
        for _ in range(sec_vein_division):
            symbols.append(F(length=0.5))
        
        symbols.append(BranchClose())
        
        leaf.extend(symbols)

    return [BranchOpen()] + leaf + [BranchClose()]


