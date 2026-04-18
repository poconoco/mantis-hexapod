from .stepper import Stepper

class StepperLinear(Stepper):
    def offset_step(self, x: float, y: float, multiplier: float) -> tuple[float, float]:
        return x * multiplier, y * multiplier
