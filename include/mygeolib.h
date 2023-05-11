#include <cmath>
#include <iostream>

class GPSConverter
{
public:
    GPSConverter()
    {
        // Set the ellipsoid parameters for WGS84
        double a = 6378137.0;
        double f = 1.0 / 298.257223563;
        double b = a * (1 - f);

        // Calculate some constants
        _e2 = (a * a - b * b) / (a * a);
        _ep2 = (_e2 * a * a) / (b * b);
        _k0 = 0.9996;
        _false_northing = 0.0;
        _false_easting = 500000.0;
    }

    void convert(double lat, double lon)
    {
        // Convert latitude and longitude to radians
        double lat_rad = lat * M_PI / 180.0;
        double lon_rad = lon * M_PI / 180.0;

        // Calculate the UTM zone and hemisphere
        int zone = static_cast<int>(std::floor((lon + 180.0) / 6.0) + 1);
        char hemisphere = lat >= 0.0 ? 'N' : 'S';

        // Calculate the central meridian
        double cm = (zone - 0.5) * 6.0 - 180.0;

        // Convert longitude to the range [-180, 180]
        double lon_norm = std::fmod(lon + 180.0, 360.0) - 180.0;

        // Calculate the meridian distance
        double s = meridianDistance(lat_rad);

        // Calculate some auxiliary quantities
        double nu2 = _e2 * std::pow(std::cos(lat_rad), 2);
        double n = std::pow(1 - _e2 * std::pow(std::sin(lat_rad), 2), -0.5);
        double t = std::tan(lat_rad) * std::tan(lat_rad);
        double c = _ep2 * std::pow(std::cos(lat_rad), 2);
        double a = std::cos(lat_rad) * (lon_rad - cm);
        double m = a * (1.0 - _e2 / 4.0 - 3.0 * _e2 * _e2 / 64.0 - 5.0 * _e2 * _e2 * _e2 / 256.0);

        // Calculate the easting and northing
        double easting = _false_easting + _k0 * n * (a + (1.0 - t + c) * std::pow(a, 3) / 6.0 + (5.0 - 18.0 * t + t * t + 72.0 * c - 58.0 * nu2) * std::pow(a, 5) / 120.0);
        double northing = _false_northing + _k0 * (s + n * std::tan(lat_rad) * (a * a / 2.0 + (5.0 - t + 9.0 * c + 4.0 * c * c) * std::pow(a, 4) / 24.0 + (61.0 - 58.0 * t + t * t + 600.0 * c - 330.0 * nu2) * std::pow(a, 6) / 720.0));
        // Adjust the northing for the southern hemisphere
        if (hemisphere == 'S')
        {
            northing += 10000000.0;
        }

        // Output the UTM coordinates
        std::cout << "Zone: " << zone << hemisphere << std::endl;
        std::cout << "Easting: " << easting << " m" << std::endl;
        std::cout << "Northing: " << northing << " m" << std::endl;
    }

private:
    double _e2;             // Ellipsoid eccentricity squared
    double _ep2;            // Ellipsoid second eccentricity squared
    double _k0;             // Scale factor at central meridian
    double _false_northing; // False northing for UTM
    double _false_easting;  // False easting for UTM
    double meridianDistance(double lat_rad)
    {
        double a = 6378137.0;
        double b = 6356752.3142;

        double e2 = (a * a - b * b) / (a * a);
        double e = std::sqrt(e2);
        double n = (a - b) / (a + b);
        double n2 = n * n;
        double n3 = n * n * n;

        double cos_lat = std::cos(lat_rad);
        double sin_lat = std::sin(lat_rad);
        double tan_lat = std::tan(lat_rad);
        double eta2 = e2 * cos_lat * cos_lat;
        double eta4 = eta2 * eta2;
        double eta6 = eta4 * eta2;
        double m = (a * ((1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256) * lat_rad - (3 * e2 / 8 + 3 * e2 * e2 / 32 + 45 * e2 * e2 * e2 / 1024) * std::sin(2 * lat_rad) + (15 * e2 * e2 / 256 + 45 * e2 * e2 * e2 / 1024) * std::sin(4 * lat_rad) - (35 * e2 * e2 * e2 / 3072) * std::sin(6 * lat_rad)));
        double T1 = (1 - n2 / 4 - 3 * n2 * n2 / 64 - 5 * n2 * n2 * n2 / 256) * lat_rad;
        double T2 = (3 * n / 2 - 27 * n3 / 32) * std::sin(2 * lat_rad);
        double T3 = (21 * n2 / 16 - 55 * n2 * n2 / 32) * std::sin(4 * lat_rad);
        double T4 = (151 * n3 / 96) * std::sin(6 * lat_rad);
        double M = b * _k0 * (T1 - T2 + T3 - T4);
        double N = a / std::sqrt(1 - e2 * sin_lat * sin_lat);
        double A = (lon_rad - ((6 * (std::floor((lon_rad + 180) / 6))) - 183) * M_PI / 180) * cos_lat;
        double T = tan_lat * tan_lat;
        double C = eta2 * cos_lat * cos_lat;
        double C2 = C * C;
        double R = (a * (1 - e2)) / std::pow(1 - e2 * sin_lat * sin_lat, 1.5);
        double D = eastingOffset(A, T, C, M);
        double D2 = D * D;

        double lat_rad_squared = lat_rad * lat_rad;
        double x = _k0 * N * (A + (1 - T + C) * D * D / 6 + (5 - 18 * T + T * T + 72 * C - 58 * eta2) * D2 * D2 / 120);

        double y = _k0 * (M + N * tan_lat * (D * D / 2 + (5 - T + 9 * C + 4 * C2) * D2 * D2 / 24 + (61 - 58 * T + T * T + 600 * C - 330 * eta2) * D2 * D2 * D2 * D2 / 720));

        return std::make_pair(x, y);
    }

    double eastingOffset(double A, double T, double C, double M)
    {
        return A + (1 - T + C) * std::pow(A, 3) / 6 + (5 - 18 * T + T * T + 72 * C - 58 * _ep2) * std::pow(A, 5) / 120;
    }

    double northingOffset(double lat_rad, double lon_rad)
    {
        double a = 6378137.0;
        double b = 6356752.3142;

        double e2 = (a * a - b * b) / (a * a);
        double e = std::sqrt(e2);
        double n = (a - b) / (a + b);
        double n2 = n * n;
        double n3 = n * n * n;

        double cos_lat = std::cos(lat_rad);
        double sin_lat = std::sin(lat_rad);
        double tan_lat = std::tan(lat_rad);
        double eta2 = e2 * cos_lat * cos_lat;
        double eta4 = eta2 * eta2;
        double eta6 = eta4 * eta2;
        double m = (a * ((1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256) * lat_rad - (3 * e2 / 8 + 3 * e2 * e2 / 32 + 45 * e2 * e2 * e2 / 1024) * std::sin(2 * lat_rad) + (15 * e2 * e2 / 256 + 45 * e2 * e2 * e2 / 1024) * std::sin(4 * lat_rad) - (35 * e2 * e2 * e2 / 3072) * std::sin(6 * lat_rad)));
        double N = a / std::sqrt(1 - e2 * sin_lat * sin_lat);
        double T = tan_lat * tan_lat;
        double C = eta2 * cos_lat * cos_lat;
        double C2 = C * C;
        double R = (a * (1 - e2)) / std::pow(1 - e2 * sin_lat * sin_lat, 1.5);
        double D = eastingOffset(A, T, C, M);
        double D2 = D * D;
        double lat_rad_squared = lat_rad * lat_rad;
        double x = _k0 * N * (A + (1 - T + C) * D2 / 6 + (5 - 18 * T + T * T + 72 * C - 58 * eta2) * D * D2 * D2 / 120);

    double y = _k0 * (M + N * tan_lat * (D2/2 + (5-T+9*C+4*C2)*D*D2*D2/24
            + (61-58*T+T*T+600*C-330*eta2)*D2*D2*D2*D2/720;
    double y_offset = _k0 * (N * tan_lat) / R * (lat_rad_squared / 2
            + (5 - T + 9*C + 4*C2) * std::pow(lat_rad_squared, 3) / 24
            + (61 - 58*T + T*T + 600*C - 330*eta2) * std::pow(lat_rad_squared, 4) / 720);

    return y + y_offset;
    }

public:
    /**
     * Constructor to initialize the UTM conversion object
     * @param lat Latitude in decimal degrees with at most 12 decimal places
     * @param lon Longitude in decimal degrees with at most 12 decimal places
     */
    void UTMConverter(double lat, double lon)
    {
    // convert lat/lon to radians
    double lat_rad = lat * DEG_TO_RAD;
    double lon_rad = lon * DEG_TO_RAD;
    // calculate zone number
    _zone = std::floor((lon + 180.0) / 6.0) + 1;

    // adjust zone number for Norway
    if (lat >= 56.0 && lat < 64.0 && lon >= 3.0 && lon < 12.0)
        _zone = 32;

    // adjust zone number for Svalbard
    if (lat >= 72.0 && lat < 84.0)
    {
        if (lon >= 0.0 && lon < 9.0)
            _zone = 31;
        else if (lon >= 9.0 && lon < 21.0)
            _zone = 33;
        else if (lon >= 21.0 && lon < 33.0)
            _zone = 35;
        else if (lon >= 33.0 && lon < 42.0)
            _zone = 37;
    }

    double lon_zone_origin = (_zone - 1) * 6.0 - 180.0 + 3.0;
    double lon_rad_origin = lon_zone_origin * DEG_TO_RAD;

    double N = 6378137.0 / std::sqrt(1 - 0.00669438002290 * std::pow(std::sin(lat_rad), 2));
    double T = std::pow(std::tan(lat_rad), 2);
    double C = 0.00673949677548 * std::pow(std::cos(lat_rad), 2);
    double A = (lon_rad - lon_rad_origin) * std::cos(lat_rad);
    double M = 0.0;

    // calculate meridian distance M
    if (lat < 0.0)
        M = 10000000.0;
    M += 6367449.14582 * (lat_rad - 0.0033242786 * std::sin(2 * lat_rad) + 0.0000053644 * std::sin(4 * lat_rad) - 0.0000000061 * std::sin(6 * lat_rad));

    double x = easting(lat_rad, A, T, C, N, M);
    double y = northing(lat_rad, lon_rad, A, T, C, N, M);

    // set UTM coordinates
    _easting = x;
    _northing = y;
    }

    /**
     * Get the UTM zone number
     * @return UTM zone number
     */
    int zone() const
    {
    return _zone;
    }

    /**
     * Get the UTM easting coordinate
     * @return
     * UTM easting coordinate
     */
    double easting() const
    {
    return _easting;
    }

    /**
     * Get the UTM northing coordinate
     * @return UTM northing coordinate
     */
    double northing() const
    {
    return _northing;
    }
};

int main()
{
    // example usage
    double lat = 48.858222;
    double lon = 2.2945;
    UTMConverter converter(lat, lon);
    std::cout << "UTM zone: " << converter.zone() << std::endl;
    std::cout << "Easting: " << converter.easting() << std::endl;
    std::cout << "Northing: " << converter.northing() << std::endl;
    return 0;
}