// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.utils;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Retention(RetentionPolicy.RUNTIME)
@Target({ElementType.FIELD, ElementType.METHOD})
public @interface AutoLogLevel {

    enum Level {
        DEBUG_SIM,
        DEBUG_REAL,
        SIM,
        SYSID,
        REAL;

        /**
         * Checks if this level is at or higher than the other level.
         * Example: {@code myLogLevel.isAtOrHigherThan(Level.SIM)} will return {@code true} if
         * {@code myLogLevel} is {@link #SIM}, {@link #SYSID}, or {@link #REAL}.
         * @param other The other level to compare against.
         * @return true if this level is at or higher than the other level.
         */
        public boolean isAtOrHigherThan(Level other) {
            return this.compareTo(other) >= 0;
        }

        /**
         * Checks if this level is at or lower than the other level.
         * Example: {@code CURRENT_LOG_LEVEL.isAtOrLowerThan(Level.SIM)} will return {@code true} if
         * {@code CURRENT_LOG_LEVEL} is {@link #SIM}, {@link #DEBUG_REAL}, or {@link #DEBUG_SIM}.
         * @param other The other level to compare against.
         * @return true if this level is at or lower than the other level.
         */
        public boolean isAtOrLowerThan(Level other) {
            return this.compareTo(other) <= 0;
        }
    }

    String key() default "";

    Level level() default Level.REAL;
}
