// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.wrappers;

import com.google.errorprone.annotations.Immutable;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Time;

@Immutable
@SuppressWarnings({"Immutable", "checkstyle", "PMD"})
public record ImmutableTime(double magnitude, double baseUnitMagnitude, TimeUnit unit) implements Time {

    public static ImmutableTime of(Time time) {
        if (time instanceof ImmutableTime it) {
            return it;
        }
        return new ImmutableTime(time.magnitude(), time.baseUnitMagnitude(), time.unit());
    }

    @Override
    public Time copy() {
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
