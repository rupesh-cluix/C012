struct DeviceInformation
{
    String deviceSerialNumber;
    String buildNumber;
    String deviceMacAddress;
    String userName;
    String phoneNumber;
    String location;
    String registrationDate;
};

DeviceInformation deviceInformationList[20] =
{
    {"C012-24A-00005", "CLX-24-BN-0005","EC:64:C9:2C:7D:C0", "Sanjay", "9822263985", "Nagpur", "17/10/24"},
    {"C012-24A-00006", "CLX-24-BN-0006","EC:64:C9:2B:CE:84", "Sanjay", "9822263985", "Nagpur", "17/10/24"},
    {"C012-24A-00007", "CLX-24-BN-0007","FC:E8:C0:D3:A7:E8", "Chetan", "7827265776", "Uttar pradesh", "17/12/24"},
    {"C012-24A-00SP Joshi050", "CLX-24-BN-0050","FC:E8:C0:D3:AF:BC", "Vikas", "9945595251", "Banglore", "16/12/24"},
    {"C012-24A-00048", "CLX-24-BN-0048","FC:E8:C0:D3:AF:2C", "Tata", "8013068581", "Guwahati", "18/12/24"},
    {"C012-24A-00049", "CLX-24-BN-0049","FC:E8:C0:D3:A7:C4", "Tata", "7005037809", "Agartala", "18/12/24"},  
    {"C012-24A-00029", "CLX-24-BN-0029","FC:E8:C0:D3:AB:8C", "GD Consult", "9945595251", "Delhi", "16/12/24"},
    {"C012-24A-00047", "CLX-24-BN-0047","FC:E8:C0:D3:B0:80", "Ambuja", "7842487976", "Sanganer", "19/12/24"},
    {"C012-24A-00046", "CLX-24-BN-0046","FC:E8:C0:D3:AB:B0", "Sudhir Ch.", "7015535963", "PHED", "04/01/25"},
    {"C012-24A-00045", "CLX-24-BN-0045","FC:E8:C0:D3:A7:FC", "Sudhir Ch.", "7015535963", "PHED", "04/01/25"},
    {"C012-24A-00044", "CLX-24-BN-0044","FC:E8:C0:D3:A8:04", "SP", "7015535963", "PHED", "04/01/25"},
    {"C012-24A-00043", "CLX-24-BN-0043","FC:E8:C0:D3:AF:38", "SID", "9999485960", "Siddharth", "11/01/25"},
    {"C012-24A-00041", "CLX-24-BN-0043","FC:E8:C0:D3:B0:28", "Amit", "1234567890", "PHED", "15/01/25"},
    {"C012-24A-00042", "CLX-24-BN-0043","FC:E8:C0:D3:AF:B4", "Amit", "1234567890", "PHED", "15/01/25"},
    {"C012-24A-00040", "CLX-24-BN-0043","FC:E8:C0:D3:AF:D0", "Amit", "1234567890", "PHED", "15/01/25"}
};

struct History
{
    int serial;
    String testName;
    String value;
    String dateTime;
    String permissible;
    String range;
    String location;
    String unit;
};

History historyList[105] = {
    {1, "Frc","54","sadfas","yes","25","aff" , "ppm"},
    {2, "Frc1","54","sadfas","yes","25","aff", "ppm"},
    {3, "Frc2","54","sadfas","yes","25","aff", "ppm"},
    {4, "Frc3","54","sadfas","yes","25","aff", "ppm"},
    {5, "Fr4","54","sadfas","yes","25","aff" , "ppm"},
    {6, "Frc5","54","sadfas","yes","25","aff", "ppm"},
    {7, "Frc6","54","sadfas","yes","25","aff", "ppm"},
    {8, "Frc7","54","sadfas","yes","25","aff", "ppm"},
    {9, "Frc8","54","sadfas","yes","25","aff", "ppm"},
    {10, "Frc","54","sadfas","yes","25","aff" , "ppm"},
    {11, "Frc","54","sadfas","yes","25","aff" , "ppm"},
    {12, "Frc1","54","sadfas","yes","25","aff", "ppm"},
    {13, "Frc2","54","sadfas","yes","25","aff", "ppm"},
    {14, "Frc3","54","sadfas","yes","25","aff", "ppm"},
    {15, "Fr4","54","sadfas","yes","25","aff" , "ppm"},
    {16, "Frc5","54","sadfas","yes","25","aff", "ppm"},
    {17, "Frc6","54","sadfas","yes","25","aff", "ppm"},
    {18, "Frc7","54","sadfas","yes","25","aff", "ppm"},
    {19, "Frc8","54","sadfas","yes","25","aff", "ppm"},
    {20, "Frc8","54","sadfas","yes","25","aff", "ppm"},
    {21, "Frc","54","sadfas","yes","25","aff" , "ppm"},
    {22, "Frc1","54","sadfas","yes","25","aff", "ppm"},
    {23, "Frc2","54","sadfas","yes","25","aff", "ppm"},
    {24, "Frc3","54","sadfas","yes","25","aff", "ppm"},
    {25, "Fr4","54","sadfas","yes","25","aff" , "ppm"},
    {26, "Frc5","54","sadfas","yes","25","aff", "ppm"},
    {27, "Frc6","54","sadfas","yes","25","aff", "ppm"},
    {28, "Frc7","54","sadfas","yes","25","aff", "ppm"},
    {29, "Frc8","54","sadfas","yes","25","aff", "ppm"},
    {30, "Frc8","54","sadfas","yes","25","aff", "ppm"},
    {31, "Frc8","54","sadfas","yes","25","aff", "ppm"},
    {32, "Frc","54","sadfas","yes","25","aff" , "ppm"},
    {33, "Frc2","54","sadfas","yes","25","aff", "ppm"},
    {34, "Frc3","54","sadfas","yes","25","aff", "ppm"},
    {35, "Fr4","54","sadfas","yes","25","aff" , "ppm"},
    {36, "Frc5","54","sadfas","yes","25","aff", "ppm"},
    {37, "Frc6","54","sadfas","yes","25","aff", "ppm"},
    {38, "Frc7","54","sadfas","yes","25","aff", "ppm"},
    {39, "Frc8","54","sadfas","yes","25","aff", "ppm"},
    {40, "Frc","54","sadfas","yes","25","aff" , "ppm"},
    {41, "Frc","54","sadfas","yes","25","aff" , "ppm"},
    {42, "Frc1","54","sadfas","yes","25","aff", "ppm"},
    {43, "Frc2","54","sadfas","yes","25","aff", "ppm"},
    {44, "Frc3","54","sadfas","yes","25","aff", "ppm"},
    {45, "Fr4","54","sadfas","yes","25","aff" , "ppm"},
    {46, "Frc5","54","sadfas","yes","25","aff", "ppm"},
    {47, "Frc6","54","sadfas","yes","25","aff", "ppm"},
    {48, "Frc7","54","sadfas","yes","25","aff", "ppm"},
    {49, "Frc8","54","sadfas","yes","25","aff", "ppm"},
    {50, "Frc","54","sadfas","yes","25","aff" , "ppm"},
    {51, "Frc","54","sadfas","yes","25","aff" , "ppm"},
    {52, "Frc1","54","sadfas","yes","25","aff", "ppm"},
    {53, "Frc2","54","sadfas","yes","25","aff", "ppm"},
    {54, "Frc3","54","sadfas","yes","25","aff", "ppm"},
    {55, "Fr4","54","sadfas","yes","25","aff" , "ppm"},
    {56, "Frc5","54","sadfas","yes","25","aff", "ppm"},
    {57, "Frc6","54","sadfas","yes","25","aff", "ppm"},
    {58, "Frc7","54","sadfas","yes","25","aff", "ppm"},
    {59, "Frc8","54","sadfas","yes","25","aff", "ppm"},
    {60, "Frc","54","sadfas","yes","25","aff" , "ppm"},
    {61, "Frc","54","sadfas","yes","25","aff" , "ppm"},
    {62, "Frc1","54","sadfas","yes","25","aff", "ppm"},
    {63, "Frc2","54","sadfas","yes","25","aff", "ppm"},
    {64, "Frc3","54","sadfas","yes","25","aff", "ppm"},
    {65, "Fr4","54","sadfas","yes","25","aff" , "ppm"},
    {66, "Frc5","54","sadfas","yes","25","aff", "ppm"},
    {67, "Frc6","54","sadfas","yes","25","aff", "ppm"},
    {68, "Frc7","54","sadfas","yes","25","aff", "ppm"},
    {69, "Frc8","54","sadfas","yes","25","aff", "ppm"},
    {70, "Frc","54","sadfas","yes","25","aff" , "ppm"},
    {71, "Frc","54","sadfas","yes","25","aff" , "ppm"},
    {72, "Frc1","54","sadfas","yes","25","aff", "ppm"},
    {73, "Frc2","54","sadfas","yes","25","aff", "ppm"},
    {74, "Frc3","54","sadfas","yes","25","aff", "ppm"},
    {75, "Fr4","54","sadfas","yes","25","aff" , "ppm"},
    {76, "Frc5","54","sadfas","yes","25","aff", "ppm"},
    {77, "Frc6","54","sadfas","yes","25","aff", "ppm"},
    {78, "Frc7","54","sadfas","yes","25","aff", "ppm"},
    {79, "Frc8","54","sadfas","yes","25","aff", "ppm"},
    {80, "Frc","54","sadfas","yes","25","aff" , "ppm"},
    {81, "Frc","54","sadfas","yes","25","aff" , "ppm"},
    {82, "Frc1","54","sadfas","yes","25","aff", "ppm"},
    {83, "Frc2","54","sadfas","yes","25","aff", "ppm"},
    {84, "Frc3","54","sadfas","yes","25","aff", "ppm"},
    {85, "Fr4","54","sadfas","yes","25","aff" , "ppm"},
    {86, "Frc5","54","sadfas","yes","25","aff", "ppm"},
    {87, "Frc6","54","sadfas","yes","25","aff", "ppm"},
    {88, "Frc7","54","sadfas","yes","25","aff", "ppm"},
    {89, "Frc8","54","sadfas","yes","25","aff", "ppm"},
    {90, "Frc","54","sadfas","yes","25","aff" , "ppm"},
    {91, "Frc","54","sadfas","yes","25","aff" , "ppm"},
    {92, "Frc1","54","sadfas","yes","25","aff", "ppm"},
    {93, "Frc2","54","sadfas","yes","25","aff", "ppm"},
    {94, "Frc3","54","sadfas","yes","25","aff", "ppm"},
    {95, "Fr4","54","sadfas","yes","25","aff" , "ppm"},
    {96, "Frc5","54","sadfas","yes","25","aff", "ppm"},
    {97, "Frc6","54","sadfas","yes","25","aff", "ppm"},
    {98, "Frc7","54","sadfas","yes","25","aff", "ppm"},
    {99, "Frc8","54","sadfas","yes","25","aff", "ppm"},
    {100, "Frc","54","sadfas","yes","25","aff" , "ppm"},
    {101, "Frc","54","sadfas","yes","25","aff" , "ppm"},
    {102, "Frc","54","sadfas","yes","25","aff" , "ppm"},
    {103, "Frc","54","sadfas","yes","25","aff" , "ppm"},
    {104, "Frc","54","sadfas","yes","25","aff" , "ppm"},
    {105, "Frc","54","sadfas","yes","25","aff" , "ppm"}
};


