#pragma once


namespace aespa_lib {
namespace units {


// ---------- Length ----------

class Length {
public:
	Length(double value_tiles);

	Length operator-();

	double tiles();
	double in();
	double qtIn();
	double cm();
	double m();

private:
	double value_tiles;
};


// ---------- Angle ----------

class Angle {
public:
	Angle(double value_fieldDeg);

	Angle operator-();

	double fieldDeg();
	double polarDeg();
	double fieldRad();
	double polarRad();

private:
	double value_fieldDeg;
};


// ---------- Literals ----------

inline namespace literals {


inline namespace length {

Length operator ""_tiles(long double value);
Length operator ""_in(long double value);
Length operator ""_qtIn(long double value);
Length operator ""_cm(long double value);
Length operator ""_m(long double value);

Length operator ""_tiles(unsigned long long value);
Length operator ""_in(unsigned long long value);
Length operator ""_qtIn(unsigned long long value);
Length operator ""_cm(unsigned long long value);
Length operator ""_m(unsigned long long value);

}


inline namespace angle {

Angle operator ""_fieldDeg(long double value);
Angle operator ""_polarDeg(long double value);
Angle operator ""_fieldRad(long double value);
Angle operator ""_polarRad(long double value);

Angle operator ""_fieldDeg(unsigned long long value);
Angle operator ""_polarDeg(unsigned long long value);
Angle operator ""_fieldRad(unsigned long long value);
Angle operator ""_polarRad(unsigned long long value);

}


}

}
}
