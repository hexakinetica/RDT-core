// Units.h
#ifndef RDT_UNITS_H
#define RDT_UNITS_H

#pragma once

#include <cmath>
#include <ostream>
#include <stdexcept>
#include <limits>
#include <string>
#include <sstream>
#include <iomanip>

namespace RDT {

// --- Constants ---
namespace UnitConstants {
    constexpr double PI = 3.1415926535897932384626433832795;
    constexpr double TWO_PI = 2.0 * PI;
    constexpr double DEFAULT_EPSILON = 1e-9; 
} // namespace UnitConstants

// --- Forward Declarations ---
class Degrees; class Radians;
class Millimeters; class Meters;
class DegreesPerSecond; class RadiansPerSecond;
class MillimetersPerSecond; class MetersPerSecond;
class DegreesPerSecondSq; class RadiansPerSecondSq;
class MillimetersPerSecondSq; class MetersPerSecondSq;
class Seconds;

// --- Helper for toString ---
template<typename T>
[[nodiscard]] inline std::string value_to_string_with_suffix(T value, int precision, const char* suffix) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value << suffix;
    return oss.str();
}


//======================================================================================
// Angle Units
//======================================================================================
class Radians {
public:
    constexpr explicit Radians(double val = 0.0) : value_(val) {}
    [[nodiscard]] constexpr double value() const { return value_; }

    Radians& operator+=(Radians other) { value_ += other.value_; return *this; }
    Radians& operator-=(Radians other) { value_ -= other.value_; return *this; }
    Radians& operator*=(double scalar) { value_ *= scalar; return *this; }
    Radians& operator/=(double scalar) {
        // В детерминированных системах исключения не используются.
        // Деление на ноль приведет к +/- INF согласно IEEE 754.
        value_ /= scalar; return *this;
    }
    [[nodiscard]] constexpr Radians operator-() const { return Radians(-value_); } // Unary minus
    [[nodiscard]] constexpr Radians abs() const { return Radians(std::abs(value_)); }

    [[nodiscard]] constexpr bool operator==(Radians other) const { return std::abs(value_ - other.value_) < UnitConstants::DEFAULT_EPSILON; }
    [[nodiscard]] constexpr bool operator!=(Radians other) const { return !(*this == other); }
    [[nodiscard]] constexpr bool operator<(Radians other) const { return value_ < other.value_ && !(*this == other); }
    [[nodiscard]] constexpr bool operator<=(Radians other) const { return value_ <= other.value_ || (*this == other); }
    [[nodiscard]] constexpr bool operator>(Radians other) const { return value_ > other.value_ && !(*this == other); }
    [[nodiscard]] constexpr bool operator>=(Radians other) const { return value_ >= other.value_ || (*this == other); }

    [[nodiscard]] Degrees toDegrees() const;
    [[nodiscard]] std::string toString(int precision = 4) const { return value_to_string_with_suffix(value_, precision, " rad"); }
    [[nodiscard]] Radians normalized() const { 
        double val = std::fmod(value_ + UnitConstants::PI, UnitConstants::TWO_PI);
        if (val < 0) val += UnitConstants::TWO_PI;
        return Radians(val - UnitConstants::PI);
    }
private: 
    double value_;
};
// Free operators for Radians
[[nodiscard]] constexpr inline Radians operator+(Radians lhs, Radians rhs) { return Radians(lhs.value() + rhs.value()); }
[[nodiscard]] constexpr inline Radians operator-(Radians lhs, Radians rhs) { return Radians(lhs.value() - rhs.value()); }
[[nodiscard]] constexpr inline Radians operator*(Radians lhs, double scalar) { return Radians(lhs.value() * scalar); }
[[nodiscard]] constexpr inline Radians operator*(double scalar, Radians rhs) { return Radians(scalar * rhs.value()); }
[[nodiscard]] inline Radians operator/(Radians lhs, double scalar) {
    return Radians(lhs.value() / scalar);
}
inline std::ostream& operator<<(std::ostream& os, Radians rad) { return os << rad.toString(); }

class Degrees {
public:
    constexpr explicit Degrees(double val = 0.0) : value_(val) {}
    [[nodiscard]] constexpr double value() const { return value_; }
    Degrees& operator+=(Degrees other) { value_ += other.value_; return *this; }
    Degrees& operator-=(Degrees other) { value_ -= other.value_; return *this; }
    Degrees& operator*=(double scalar) { value_ *= scalar; return *this; }
    Degrees& operator/=(double scalar) {
        value_ /= scalar; return *this;
    }
    [[nodiscard]] constexpr Degrees operator-() const { return Degrees(-value_); }
    [[nodiscard]] constexpr Degrees abs() const { return Degrees(std::abs(value_)); }

    [[nodiscard]] constexpr bool operator==(Degrees other) const { return std::abs(value_ - other.value_) < UnitConstants::DEFAULT_EPSILON; }
    [[nodiscard]] constexpr bool operator!=(Degrees other) const { return !(*this == other); }
    [[nodiscard]] constexpr bool operator<(Degrees other) const { return value_ < other.value_ && !(*this == other); }
    [[nodiscard]] constexpr bool operator<=(Degrees other) const { return value_ <= other.value_ || (*this == other); }
    [[nodiscard]] constexpr bool operator>(Degrees other) const { return value_ > other.value_ && !(*this == other); }
    [[nodiscard]] constexpr bool operator>=(Degrees other) const { return value_ >= other.value_ || (*this == other); }

    [[nodiscard]] Radians toRadians() const;
    [[nodiscard]] std::string toString(int precision = 2) const { return value_to_string_with_suffix(value_, precision, " deg"); }
    [[nodiscard]] Degrees normalized() const {
        double val = std::fmod(value_ + 180.0, 360.0);
        if (val < 0) val += 360.0;
        return Degrees(val - 180.0);
    }
private: 
    double value_;
};
[[nodiscard]] constexpr inline Degrees operator+(Degrees lhs, Degrees rhs) { return Degrees(lhs.value() + rhs.value()); }
[[nodiscard]] constexpr inline Degrees operator-(Degrees lhs, Degrees rhs) { return Degrees(lhs.value() - rhs.value()); }
[[nodiscard]] constexpr inline Degrees operator*(Degrees lhs, double scalar) { return Degrees(lhs.value() * scalar); }
[[nodiscard]] constexpr inline Degrees operator*(double scalar, Degrees rhs) { return Degrees(scalar * rhs.value()); }
[[nodiscard]] inline Degrees operator/(Degrees lhs, double scalar) {
    return Degrees(lhs.value() / scalar);
}
inline std::ostream& operator<<(std::ostream& os, Degrees deg) { return os << deg.toString(); }
// Conversions defined after both classes are complete
inline Degrees Radians::toDegrees() const { return Degrees(value_ * (180.0 / UnitConstants::PI)); }
inline Radians Degrees::toRadians() const { return Radians(value_ * (UnitConstants::PI / 180.0)); }


//======================================================================================
// Linear Distance Units
//======================================================================================
class Meters {
public:
    constexpr explicit Meters(double val = 0.0) : value_(val) {}
    [[nodiscard]] constexpr double value() const { return value_; }
    Meters& operator+=(Meters other) { value_ += other.value_; return *this; }
    Meters& operator-=(Meters other) { value_ -= other.value_; return *this; }
    Meters& operator*=(double scalar) { value_ *= scalar; return *this; }
    Meters& operator/=(double scalar) { value_ /= scalar; return *this;}
    [[nodiscard]] constexpr Meters operator-() const { return Meters(-value_); }
    [[nodiscard]] constexpr Meters abs() const { return Meters(std::abs(value_)); }
    [[nodiscard]] constexpr bool operator==(Meters other) const { return std::abs(value_ - other.value_) < UnitConstants::DEFAULT_EPSILON; }
    [[nodiscard]] constexpr bool operator!=(Meters other) const { return !(*this == other); }
    [[nodiscard]] constexpr bool operator<(Meters other) const { return value_ < other.value_ && !(*this == other); }
    [[nodiscard]] constexpr bool operator<=(Meters other) const { return value_ <= other.value_ || (*this == other); }
    [[nodiscard]] constexpr bool operator>(Meters other) const { return value_ > other.value_ && !(*this == other); }
    [[nodiscard]] constexpr bool operator>=(Meters other) const { return value_ >= other.value_ || (*this == other); }
    [[nodiscard]] Millimeters toMillimeters() const;
    [[nodiscard]] std::string toString(int precision = 4) const { return value_to_string_with_suffix(value_, precision, " m"); }
private: double value_;
};
[[nodiscard]] constexpr inline Meters operator+(Meters lhs, Meters rhs) { return Meters(lhs.value() + rhs.value()); }
[[nodiscard]] constexpr inline Meters operator-(Meters lhs, Meters rhs) { return Meters(lhs.value() - rhs.value()); }
[[nodiscard]] constexpr inline Meters operator*(Meters lhs, double scalar) { return Meters(lhs.value() * scalar); }
[[nodiscard]] constexpr inline Meters operator*(double scalar, Meters rhs) { return Meters(scalar * rhs.value()); }
[[nodiscard]] inline Meters operator/(Meters lhs, double scalar) { return Meters(lhs.value() / scalar); }
inline std::ostream& operator<<(std::ostream& os, Meters m) { return os << m.toString(); }

class Millimeters {
public:
    constexpr explicit Millimeters(double val = 0.0) : value_(val) {}
    [[nodiscard]] constexpr double value() const { return value_; }
    Millimeters& operator+=(Millimeters other) { value_ += other.value_; return *this; }
    Millimeters& operator-=(Millimeters other) { value_ -= other.value_; return *this; }
    Millimeters& operator*=(double scalar) { value_ *= scalar; return *this; }
    Millimeters& operator/=(double scalar) { value_ /= scalar; return *this;}
    [[nodiscard]] constexpr Millimeters operator-() const { return Millimeters(-value_); }
    [[nodiscard]] constexpr Millimeters abs() const { return Millimeters(std::abs(value_)); }
    [[nodiscard]] constexpr bool operator==(Millimeters other) const { return std::abs(value_ - other.value_) < UnitConstants::DEFAULT_EPSILON; }
    // ... other comparisons
    [[nodiscard]] Meters toMeters() const { return Meters(value_ / 1000.0); }
    [[nodiscard]] std::string toString(int precision = 1) const { return value_to_string_with_suffix(value_, precision, " mm"); }
private: double value_;
};
[[nodiscard]] constexpr inline Millimeters operator+(Millimeters lhs, Millimeters rhs) { return Millimeters(lhs.value()+rhs.value());}
inline std::ostream& operator<<(std::ostream& os, Millimeters mm) { return os << mm.toString(); }
inline Millimeters Meters::toMillimeters() const { return Millimeters(value_ * 1000.0); }


//======================================================================================
// Time Units
//======================================================================================
class Seconds {
public:
    constexpr explicit Seconds(double val = 0.0) : value_(val) {}
    [[nodiscard]] constexpr double value() const { return value_; }
    Seconds& operator+=(Seconds other) { value_ += other.value_; return *this; }
    Seconds& operator-=(Seconds other) { value_ -= other.value_; return *this; }
    Seconds& operator*=(double scalar) { value_ *= scalar; return *this; }
    Seconds& operator/=(double scalar) { value_ /= scalar; return *this;}
    [[nodiscard]] constexpr Seconds operator-() const { return Seconds(-value_); }
    [[nodiscard]] constexpr Seconds abs() const { return Seconds(std::abs(value_)); }
    [[nodiscard]] constexpr bool operator==(Seconds other) const { return std::abs(value_ - other.value_) < UnitConstants::DEFAULT_EPSILON; }
    [[nodiscard]] constexpr bool operator!=(Seconds other) const { return !(*this == other); }
    [[nodiscard]] constexpr bool operator<(Seconds other) const { return value_ < other.value_ && !(*this == other); }
    [[nodiscard]] constexpr bool operator<=(Seconds other) const { return value_ <= other.value_ || (*this == other); }
    [[nodiscard]] constexpr bool operator>(Seconds other) const { return value_ > other.value_ && !(*this == other); }
    [[nodiscard]] constexpr bool operator>=(Seconds other) const { return value_ >= other.value_ || (*this == other); }
    [[nodiscard]] std::string toString(int precision = 3) const { return value_to_string_with_suffix(value_, precision, " s"); }
private: double value_;
};
[[nodiscard]] constexpr inline Seconds operator+(Seconds lhs, Seconds rhs) { return Seconds(lhs.value() + rhs.value()); }
[[nodiscard]] constexpr inline Seconds operator-(Seconds lhs, Seconds rhs) { return Seconds(lhs.value() - rhs.value()); }
[[nodiscard]] constexpr inline Seconds operator*(Seconds lhs, double scalar) { return Seconds(lhs.value() * scalar); }
[[nodiscard]] constexpr inline Seconds operator*(double scalar, Seconds rhs) { return Seconds(scalar * rhs.value()); }
[[nodiscard]] inline Seconds operator/(Seconds lhs, double scalar) { return Seconds(lhs.value() / scalar); }
inline std::ostream& operator<<(std::ostream& os, Seconds s) { return os << s.toString(); }


//======================================================================================
// Velocity & Acceleration Units (Angular)
//======================================================================================
#define DEFINE_DERIVED_UNIT_OPERATIONS(ClassName) \
public: \
    constexpr explicit ClassName(double val = 0.0) : value_(val) {} \
    [[nodiscard]] constexpr double value() const { return value_; } \
    ClassName& operator+=(ClassName other) { value_ += other.value_; return *this; } \
    ClassName& operator-=(ClassName other) { value_ -= other.value_; return *this; } \
    ClassName& operator*=(double scalar) { value_ *= scalar; return *this; } \
    ClassName& operator/=(double scalar) { value_ /= scalar; return *this; } \
    [[nodiscard]] constexpr ClassName operator-() const { return ClassName(-value_); } \
    [[nodiscard]] constexpr ClassName abs() const { return ClassName(std::abs(value_)); } \
    [[nodiscard]] constexpr bool operator==(ClassName other) const { return std::abs(value_ - other.value_) < UnitConstants::DEFAULT_EPSILON; } \
    [[nodiscard]] constexpr bool operator!=(ClassName other) const { return !(*this == other); } \
    [[nodiscard]] constexpr bool operator<(ClassName other) const { return value_ < other.value_ && !(*this == other); } \
    [[nodiscard]] constexpr bool operator<=(ClassName other) const { return value_ <= other.value_ || (*this == other); } \
    [[nodiscard]] constexpr bool operator>(ClassName other) const { return value_ > other.value_ && !(*this == other); } \
    [[nodiscard]] constexpr bool operator>=(ClassName other) const { return value_ >= other.value_ || (*this == other); } \
private: \
    double value_; \
public: \
    friend constexpr inline ClassName operator+(ClassName lhs, ClassName rhs) { return ClassName(lhs.value() + rhs.value()); } \
    friend constexpr inline ClassName operator-(ClassName lhs, ClassName rhs) { return ClassName(lhs.value() - rhs.value()); } \
    friend constexpr inline ClassName operator*(ClassName lhs, double scalar) { return ClassName(lhs.value() * scalar); } \
    friend constexpr inline ClassName operator*(double scalar, ClassName rhs) { return ClassName(scalar * rhs.value()); } \
    friend inline ClassName operator/(ClassName lhs, double scalar) { return ClassName(lhs.value() / scalar); }

class RadiansPerSecond {
    DEFINE_DERIVED_UNIT_OPERATIONS(RadiansPerSecond)
    [[nodiscard]] DegreesPerSecond toDegreesPerSecond() const;
    [[nodiscard]] std::string toString(int precision = 3) const { return value_to_string_with_suffix(value_, precision, " rad/s"); }
};
inline std::ostream& operator<<(std::ostream& os, RadiansPerSecond rps) { return os << rps.toString(); }

class DegreesPerSecond {
    DEFINE_DERIVED_UNIT_OPERATIONS(DegreesPerSecond)
    [[nodiscard]] RadiansPerSecond toRadiansPerSecond() const;
    [[nodiscard]] std::string toString(int precision = 1) const { return value_to_string_with_suffix(value_, precision, " deg/s"); }
};
inline std::ostream& operator<<(std::ostream& os, DegreesPerSecond dps) { return os << dps.toString(); }
inline DegreesPerSecond RadiansPerSecond::toDegreesPerSecond() const { return DegreesPerSecond(value_ * (180.0 / UnitConstants::PI)); }
inline RadiansPerSecond DegreesPerSecond::toRadiansPerSecond() const { return RadiansPerSecond(value_ * (UnitConstants::PI / 180.0)); }

class RadiansPerSecondSq {
    DEFINE_DERIVED_UNIT_OPERATIONS(RadiansPerSecondSq)
    [[nodiscard]] DegreesPerSecondSq toDegreesPerSecondSq() const;
    [[nodiscard]] std::string toString(int precision = 3) const { return value_to_string_with_suffix(value_, precision, " rad/s^2"); }
};
inline std::ostream& operator<<(std::ostream& os, RadiansPerSecondSq rps2) { return os << rps2.toString(); }

class DegreesPerSecondSq {
    DEFINE_DERIVED_UNIT_OPERATIONS(DegreesPerSecondSq)
    [[nodiscard]] RadiansPerSecondSq toRadiansPerSecondSq() const;
    [[nodiscard]] std::string toString(int precision = 1) const { return value_to_string_with_suffix(value_, precision, " deg/s^2"); }
};
inline std::ostream& operator<<(std::ostream& os, DegreesPerSecondSq dps2) { return os << dps2.toString(); }
inline DegreesPerSecondSq RadiansPerSecondSq::toDegreesPerSecondSq() const { return DegreesPerSecondSq(value_ * (180.0 / UnitConstants::PI)); }
inline RadiansPerSecondSq DegreesPerSecondSq::toRadiansPerSecondSq() const { return RadiansPerSecondSq(value_ * (UnitConstants::PI / 180.0)); }


//======================================================================================
// Velocity & Acceleration Units (Linear)
//======================================================================================
class MetersPerSecond {
    DEFINE_DERIVED_UNIT_OPERATIONS(MetersPerSecond)
    [[nodiscard]] MillimetersPerSecond toMillimetersPerSecond() const;
    [[nodiscard]] std::string toString(int precision = 4) const { return value_to_string_with_suffix(value_, precision, " m/s"); }
};
inline std::ostream& operator<<(std::ostream& os, MetersPerSecond mps) { return os << mps.toString(); }

class MillimetersPerSecond {
    DEFINE_DERIVED_UNIT_OPERATIONS(MillimetersPerSecond)
    [[nodiscard]] MetersPerSecond toMetersPerSecond() const { return MetersPerSecond(value_ / 1000.0); }
    [[nodiscard]] std::string toString(int precision = 1) const { return value_to_string_with_suffix(value_, precision, " mm/s"); }
};
inline std::ostream& operator<<(std::ostream& os, MillimetersPerSecond mmps) { return os << mmps.toString(); }
inline MillimetersPerSecond MetersPerSecond::toMillimetersPerSecond() const { return MillimetersPerSecond(value_ * 1000.0); }

class MetersPerSecondSq {
    DEFINE_DERIVED_UNIT_OPERATIONS(MetersPerSecondSq)
    [[nodiscard]] MillimetersPerSecondSq toMillimetersPerSecondSq() const;
    [[nodiscard]] std::string toString(int precision = 4) const { return value_to_string_with_suffix(value_, precision, " m/s^2"); }
};
inline std::ostream& operator<<(std::ostream& os, MetersPerSecondSq mps2) { return os << mps2.toString(); }

class MillimetersPerSecondSq {
    DEFINE_DERIVED_UNIT_OPERATIONS(MillimetersPerSecondSq)
    [[nodiscard]] MetersPerSecondSq toMetersPerSecondSq() const;
    [[nodiscard]] std::string toString(int precision = 1) const { return value_to_string_with_suffix(value_, precision, " mm/s^2"); }
};
inline std::ostream& operator<<(std::ostream& os, MillimetersPerSecondSq mmps2) { return os << mmps2.toString(); }
inline MillimetersPerSecondSq MetersPerSecondSq::toMillimetersPerSecondSq() const { return MillimetersPerSecondSq(value_ * 1000.0); }
inline MetersPerSecondSq MillimetersPerSecondSq::toMetersPerSecondSq() const { return MetersPerSecondSq(value_ / 1000.0); }


#undef DEFINE_DERIVED_UNIT_OPERATIONS // Clean up macro

// --- Inter-Unit Operations ---
[[nodiscard]] constexpr inline MetersPerSecond operator/(Meters dist, Seconds time) {
    return MetersPerSecond(dist.value() / time.value());
}
[[nodiscard]] constexpr inline RadiansPerSecond operator/(Radians angle, Seconds time) {
    return RadiansPerSecond(angle.value() / time.value());
}
[[nodiscard]] constexpr inline Meters operator*(MetersPerSecond vel, Seconds time) { return Meters(vel.value() * time.value()); }
[[nodiscard]] constexpr inline Meters operator*(Seconds time, MetersPerSecond vel) { return Meters(time.value() * vel.value()); }
[[nodiscard]] constexpr inline Radians operator*(RadiansPerSecond vel, Seconds time) { return Radians(vel.value() * time.value()); }
[[nodiscard]] constexpr inline Radians operator*(Seconds time, RadiansPerSecond vel) { return Radians(time.value() * vel.value()); }

[[nodiscard]] constexpr inline MetersPerSecondSq operator/(MetersPerSecond vel, Seconds time) {
    return MetersPerSecondSq(vel.value() / time.value());
}
[[nodiscard]] constexpr inline RadiansPerSecondSq operator/(RadiansPerSecond vel, Seconds time) {
    return RadiansPerSecondSq(vel.value() / time.value());
}
[[nodiscard]] constexpr inline MetersPerSecond operator*(MetersPerSecondSq accel, Seconds time) { return MetersPerSecond(accel.value() * time.value()); }
[[nodiscard]] constexpr inline MetersPerSecond operator*(Seconds time, MetersPerSecondSq accel) { return MetersPerSecond(time.value() * accel.value()); }
[[nodiscard]] constexpr inline RadiansPerSecond operator*(RadiansPerSecondSq accel, Seconds time) { return RadiansPerSecond(accel.value() * time.value()); }
[[nodiscard]] constexpr inline RadiansPerSecond operator*(Seconds time, RadiansPerSecondSq accel) { return RadiansPerSecond(time.value() * accel.value()); }

// User-defined literals
namespace literals {
    [[nodiscard]] constexpr Radians operator"" _rad(long double val) { return Radians(static_cast<double>(val)); }
    [[nodiscard]] constexpr Degrees operator"" _deg(long double val) { return Degrees(static_cast<double>(val)); }
    [[nodiscard]] constexpr Meters operator"" _m(long double val) { return Meters(static_cast<double>(val)); }
    [[nodiscard]] constexpr Millimeters operator"" _mm(long double val) { return Millimeters(static_cast<double>(val)); }
    [[nodiscard]] constexpr Seconds operator"" _s(long double val) { return Seconds(static_cast<double>(val)); }
    
    [[nodiscard]] constexpr RadiansPerSecond operator"" _rad_s(long double val) { return RadiansPerSecond(static_cast<double>(val)); }
    [[nodiscard]] constexpr DegreesPerSecond operator"" _deg_s(long double val) { return DegreesPerSecond(static_cast<double>(val)); }
    [[nodiscard]] constexpr MetersPerSecond operator"" _m_s(long double val) { return MetersPerSecond(static_cast<double>(val)); }
    [[nodiscard]] constexpr MillimetersPerSecond operator"" _mm_s(long double val) { return MillimetersPerSecond(static_cast<double>(val)); }

    [[nodiscard]] constexpr RadiansPerSecondSq operator"" _rad_s2(long double val) { return RadiansPerSecondSq(static_cast<double>(val)); }
    [[nodiscard]] constexpr DegreesPerSecondSq operator"" _deg_s2(long double val) { return DegreesPerSecondSq(static_cast<double>(val)); }
    [[nodiscard]] constexpr MetersPerSecondSq operator"" _m_s2(long double val) { return MetersPerSecondSq(static_cast<double>(val)); }
    [[nodiscard]] constexpr MillimetersPerSecondSq operator"" _mm_s2(long double val) { return MillimetersPerSecondSq(static_cast<double>(val)); }
} // namespace literals

} // namespace RDT
#endif // RDT_UNITS_H