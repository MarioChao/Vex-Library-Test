#include "Aespa-Lib/Winter-Utilities/units.h"

#include "Aespa-Lib/Winter-Utilities/general.h"


namespace {
const double inches_per_tile = 23 + 13.0 / 16;
}


namespace aespa_lib {
namespace units {


// ---------- Length ----------

Length::Length(double value_tiles) : value_tiles(value_tiles) {}

Length Length::operator-() { return Length(-value_tiles); }

double Length::tiles() { return value_tiles; }
double Length::in() { return tiles() * inches_per_tile; }
double Length::qtIn() { return in() * 4; }
double Length::cm() { return in() * 2.54; }
double Length::m() { return cm() / 100; }


// ---------- Angle ----------

Angle::Angle(double value_fieldDeg) : value_fieldDeg(value_fieldDeg) {}

Angle Angle::operator-() { return Angle(-value_fieldDeg); }

double Angle::fieldDeg() { return value_fieldDeg; }
double Angle::polarDeg() { return 90 - fieldDeg(); }
double Angle::fieldRad() { return genutil::toRadians(fieldDeg()); }
double Angle::polarRad() { return genutil::toRadians(polarDeg()); }


// ---------- Literals ----------

inline namespace literals {


inline namespace length {

Length operator ""_tiles(long double value) { return Length(value); }
Length operator ""_in(long double value) { return operator ""_tiles(value / inches_per_tile); }
Length operator ""_qtIn(long double value) { return operator ""_in(value / 4); }
Length operator ""_cm(long double value) { return operator ""_in(value / 2.54); }
Length operator ""_m(long double value) { return operator ""_cm(value * 100); }

Length operator ""_tiles(unsigned long long value) { return operator ""_tiles((long double) value); }
Length operator ""_in(unsigned long long value) { return operator ""_in((long double) value); }
Length operator ""_qtIn(unsigned long long value) { return operator ""_qtIn((long double) value); }
Length operator ""_cm(unsigned long long value) { return operator ""_cm((long double) value); }
Length operator ""_m(unsigned long long value) { return operator ""_m((long double) value); }

}


inline namespace angle {

Angle operator ""_fieldDeg(long double value) { return Angle(value); }
Angle operator ""_polarDeg(long double value) { return operator ""_fieldDeg(90 - value); }
Angle operator ""_fieldRad(long double value) { return operator ""_fieldDeg((long double) genutil::toDegrees(value)); }
Angle operator ""_polarRad(long double value) { return operator ""_polarDeg((long double) genutil::toDegrees(value)); }

Angle operator ""_fieldDeg(unsigned long long value) { return operator ""_fieldDeg((long double) value); }
Angle operator ""_polarDeg(unsigned long long value) { return operator ""_polarDeg((long double) value); }
Angle operator ""_fieldRad(unsigned long long value) { return operator ""_fieldRad((long double) value); }
Angle operator ""_polarRad(unsigned long long value) { return operator ""_polarRad((long double) value); }

}


}

}
}
