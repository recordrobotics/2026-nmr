// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.wrappers;

import com.google.errorprone.annotations.Immutable;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Current;

@Immutable
@SuppressWarnings({"Immutable", "checkstyle", "PMD"})
public record ImmutableCurrent(double magnitude, double baseUnitMagnitude, CurrentUnit unit) implements Current {

    public static ImmutableCurrent of(Current current) {
        if (current instanceof ImmutableCurrent ic) {
            return ic;
        }
        return new ImmutableCurrent(current.magnitude(), current.baseUnitMagnitude(), current.unit());
    }

    @Override
    public Current copy() {
        return this;
    }

    @Override
    public String toString() {
        return toShortString();
    }

    @Override
    public boolean equals(Object o) {
        return o instanceof Measure<?> m && isEquivalent(m);
    }
}
