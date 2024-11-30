import numpy as np
from dataclasses import dataclass

@dataclass
class NormalDistribution:
    """
    Normal distribution based random number generator that can be configured with a mean and standard deviation.

    Attributes:
    mean (float): The mean of the normal distribution.
    std (float): The standard deviation of the normal distribution.
    """
    mean: float
    std: float

    def sample(self):
        # TODO: Add option to seed the random number generator.
        return np.random.normal(self.mean, self.std)
    


