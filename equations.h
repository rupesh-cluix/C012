// FRC Test
#define FRC_SLOPE 2.1928
#define FRC_INTERCEPT -0.0932

// TDS Test
#define TDS_CONDUCTIVITY_FACTOR 0.0721
// relaxation_factor=EC/captured_frequency


#define TDS_SLOPE 0.1503
#define TDs_INTERCEPT -69.4977

// Y= 0.1503x-69.4977 

// Iron TestSA
// Iron TestSA

#define IRON_SLOPE 1.9786
#define IRON_INTERCEPT 0.0771

// pH Test
#define PH_SLOPE 3.4904
#define PH_INTERCEPT 5.6087

// Total Hardness (TH) Test
#define TH_LOW_CONCENTRATION 3.5
#define TH_HIGH_CONCENTRATION 35

// Phosphate Test
#define PHOSPHATE_SLOPE 2.8141
#define PHOSPHATE_INTERCEPT -0.1807
// #define PHOSPHATE_SLOPE 2.9199
// #define PHOSPHATE_INTERCEPT -0.2579

// Total Alkalinity (TA) Test
#define TA_LOW_CONCENTRATION 3.5
#define TA_HIGH_CONCENTRATION 35

// Nitrate Test
#define NITRATE_SLOPE 33.4065
#define NITRATE_INTERCEPT 3.2332

// Fluoride Test
// #define FLUORIDE_SLOPE 1.369
// #define FLUORIDE_INTERCEPT -1.5283
#define FLUORIDE_SLOPE 1.1265
#define FLUORIDE_INTERCEPT -1.1895


// Copper Test
#define COPPER_SLOPE 5.9395
#define COPPER_INTERCEPT 0.0142


// Define wavelengths
#define FRC_WAVELENGTH 530
#define IRON_WAVELENGTH 508
#define PH_WAVELENGTH_450 450
#define PH_WAVELENGTH_520 520
#define PHOSPHATE_WAVELENGTH 699
#define NITRATE_WAVELENGTH 540
#define FLUORIDE_WAVELENGTH 520
//#define FLUORIDE_WAVELENGTH 699
// #define FLUORIDE_WAVELENGTH 550
#define COPPER_WAVELENGTH 484

// Define filter settings
#define FRC_FILTER "f4"
#define IRON_FILTER "f4"
#define PH_FILTER_450 "f4"
#define PH_FILTER_520 "f4"
#define PHOSPHATE_FILTER "f6"
#define NITRATE_FILTER "f4"
#define FLUORIDE_FILTER "f4"
#define COPPER_FILTER "f2"

struct Turbidity {
  String deviceNo;
  String mac_Address;
  float slope;
  float intercept;
};

const int numberOfTurb = 50;
Turbidity turbidityList[numberOfTurb] = {
  { "LC9", "FC:E8:C0:D3:AF:2C", 0.0534, -11.7398 },
  { "LC10", "FC:E8:C0:D3:A7:C4", 0.0595, -13.0734 },
  { "LC6", "FC:E8:C0:D3:AB:8C", 0.067, -6.7861 },
  { "LC3", "FC:E8:C0:D3:AF:F0", 0.039, -4.4276},
  { "LC4", "FC:E8:C0:D3:B0:80", 0.0793, -18.3973 },
  { "LC11", "FC:E8:C0:D3:A8:04", 0.0305, -3.9437 },
  { "LC12", "FC:E8:C0:D3:AB:B0", 0.0321, -6.9394},
  { "LC2", "FC:E8:C0:D3:A7:FC", 0.0408, -6.0},
  { "LC13", "FC:E8:C0:D3:AF:38", 0.0375, -8.6768},
  { "LC16", "FC:E8:C0:D3:B0:28", 0.0291, -9.1909},
  { "LC15", "FC:E8:C0:D3:AF:B4", 0.0421, -8.9294},
  { "LC14", "FC:E8:C0:D3:AF:D0", 0.0401, -4.6386}
};

struct TDS {
  String deviceNo;
  String mac_Address;
  float factor1;
  float factor2;
  float factor3;
};

TDS tdsList[numberOfTurb] = {
  { "LC3", "FC:E8:C0:D3:AF:F0", 0.07962085308, 0.095, 0.1},
  { "LC11", "FC:E8:C0:D3:A8:04", 0.0752143241, 0.085, 0.1},//FC:E8:C0:D3:A8:04  SN.44
  { "LC12", "FC:E8:C0:D3:AB:B0", 0.06829268293, 0.08672965428, 0.09365364416},//FC:E8:C0:D3:AB:B0 SN.46
  { "LC2", "FC:E8:C0:D3:A7:FC", 0.076, 0.085, 0.09972579016} //FC:E8:C0:D3:A7:FC SN.45
};