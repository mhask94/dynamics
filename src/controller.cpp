#include "controller.hpp"

Controller::Controller()
{
    load_data();
    set_defaults();
    setup_indexing();
}

Controller::~Controller()
{

}

dyn::uVec Controller::calculateControl(const dyn::xVec &states)
{
    double u_equilibrium{0.55};
    dyn::uVec inputs{u_equilibrium,u_equilibrium,u_equilibrium,u_equilibrium};
    return inputs;
}

void Controller::load_data()
{
    m_params.x_0[0] = 0.0;
    m_params.x_0[1] = 0.0;
    m_params.x_0[2] = 0.0;
    m_params.x_0[3] = 0.0;
    m_params.x_0[4] = 0.0;
    m_params.x_0[5] = 0.0;
    m_params.x_0[6] = 0.0;
    m_params.x_0[7] = 0.0;
    m_params.x_0[8] = 0.0;
    m_params.x_0[9] = 0.0;
    m_params.x_0[10] = 0.0;
    m_params.x_0[11] = 0.0;
    m_params.x_des_0[0] = 0.0;
    m_params.x_des_0[1] = 0.0;
    m_params.x_des_0[2] = 0.0;
    m_params.x_des_0[3] = 0.0;
    m_params.x_des_0[4] = 0.0;
    m_params.x_des_0[5] = 0.0;
    m_params.x_des_0[6] = 0.0;
    m_params.x_des_0[7] = 0.0;
    m_params.x_des_0[8] = 0.0;
    m_params.x_des_0[9] = 0.0;
    m_params.x_des_0[10] = 0.0;
    m_params.x_des_0[11] = 0.0;
    m_params.Wy[0] = 1.5;
    m_params.Wy[1] = 1.5;
    m_params.Wy[2] = 20.0;
    m_params.Wy[3] = 1.0;
    m_params.Wy[4] = 1.0;
    m_params.Wy[5] = 5.0;
    m_params.Wy[6] = 2.0;
    m_params.Wy[7] = 2.0;
    m_params.Wy[8] = 2.0;
    m_params.Wy[9] = 50.0;
    m_params.Wy[10] = 50.0;
    m_params.Wy[11] = 1.0;
    m_params.u_des_0[0] = 0.6136436100941447;
    m_params.u_des_0[1] = 0.2313630495538037;
    m_params.u_des_0[2] = -0.5537409477496875;
    m_params.u_des_0[3] = -1.0997819806406723;
    m_params.Wu[0] = 1.4065199163762485;
    m_params.Wu[1] = 1.4689402486991692;
    m_params.Wu[2] = 1.2692355782510614;
    m_params.Wu[3] = 1.2917927742254327;
    m_params.x_des_1[0] = -0.16925440270808823;
    m_params.x_des_1[1] = 1.442135651787706;
    m_params.x_des_1[2] = 0.34501161787128565;
    m_params.x_des_1[3] = -0.8660485502711608;
    m_params.x_des_1[4] = -0.8880899735055947;
    m_params.x_des_1[5] = -0.1815116979122129;
    m_params.x_des_1[6] = -1.17835862158005;
    m_params.x_des_1[7] = -1.1944851558277074;
    m_params.x_des_1[8] = 0.05614023926976763;
    m_params.x_des_1[9] = -1.6510825248767813;
    m_params.x_des_1[10] = -0.06565787059365391;
    m_params.x_des_1[11] = -0.5512951504486665;
    m_params.u_des_1[0] = 0.8307464872626844;
    m_params.u_des_1[1] = 0.9869848924080182;
    m_params.u_des_1[2] = 0.7643716874230573;
    m_params.u_des_1[3] = 0.7567216550196565;
    m_params.x_des_2[0] = -0.5055995034042868;
    m_params.x_des_2[1] = 0.6725392189410702;
    m_params.x_des_2[2] = -0.6406053441727284;
    m_params.x_des_2[3] = 0.29117547947550015;
    m_params.x_des_2[4] = -0.6967713677405021;
    m_params.x_des_2[5] = -0.21941980294587182;
    m_params.x_des_2[6] = -1.753884276680243;
    m_params.x_des_2[7] = -1.0292983112626475;
    m_params.x_des_2[8] = 1.8864104246942706;
    m_params.x_des_2[9] = -1.077663182579704;
    m_params.x_des_2[10] = 0.7659100437893209;
    m_params.x_des_2[11] = 0.6019074328549583;
    m_params.u_des_2[0] = 0.8957565577499285;
    m_params.u_des_2[1] = -0.09964555746227477;
    m_params.u_des_2[2] = 0.38665509840745127;
    m_params.u_des_2[3] = -1.7321223042686946;
    m_params.x_des_3[0] = -1.7097514487110663;
    m_params.x_des_3[1] = -1.2040958948116867;
    m_params.x_des_3[2] = -1.3925560119658358;
    m_params.x_des_3[3] = -1.5995826216742213;
    m_params.x_des_3[4] = -1.4828245415645833;
    m_params.x_des_3[5] = 0.21311092723061398;
    m_params.x_des_3[6] = -1.248740700304487;
    m_params.x_des_3[7] = 1.808404972124833;
    m_params.x_des_3[8] = 0.7264471152297065;
    m_params.x_des_3[9] = 0.16407869343908477;
    m_params.x_des_3[10] = 0.8287224032315907;
    m_params.x_des_3[11] = -0.9444533161899464;
    m_params.u_des_3[0] = 1.7069027370149112;
    m_params.u_des_3[1] = 1.3567722311998827;
    m_params.u_des_3[2] = 0.9052779937121489;
    m_params.u_des_3[3] = -0.07904017565835986;
    m_params.x_des_4[0] = 1.3684127435065871;
    m_params.x_des_4[1] = 0.979009293697437;
    m_params.x_des_4[2] = 0.6413036255984501;
    m_params.x_des_4[3] = 1.6559010680237511;
    m_params.x_des_4[4] = 0.5346622551502991;
    m_params.x_des_4[5] = -0.5362376605895625;
    m_params.x_des_4[6] = 0.2113782926017822;
    m_params.x_des_4[7] = -1.2144776931994525;
    m_params.x_des_4[8] = -1.2317108144255875;
    m_params.x_des_4[9] = 0.9026784957312834;
    m_params.x_des_4[10] = 1.1397468137245244;
    m_params.x_des_4[11] = 1.8883934547350631;
    m_params.u_des_4[0] = 1.4038856681660068;
    m_params.u_des_4[1] = 0.17437730638329096;
    m_params.u_des_4[2] = -1.6408365219077408;
    m_params.u_des_4[3] = -0.04450702153554875;
    m_params.x_des_5[0] = 1.7117453902485025;
    m_params.x_des_5[1] = 1.1504727980139053;
    m_params.x_des_5[2] = -0.05962309578364744;
    m_params.x_des_5[3] = -0.1788825540764547;
    m_params.x_des_5[4] = -1.1280569263625857;
    m_params.x_des_5[5] = -1.2911464767927057;
    m_params.x_des_5[6] = -1.7055053231225696;
    m_params.x_des_5[7] = 1.56957275034837;
    m_params.x_des_5[8] = 0.5607064675962357;
    m_params.x_des_5[9] = -1.4266707301147146;
    m_params.x_des_5[10] = -0.3434923211351708;
    m_params.x_des_5[11] = -1.8035643024085055;
    m_params.u_des_5[0] = -1.1625066019105454;
    m_params.u_des_5[1] = 0.9228324965161532;
    m_params.u_des_5[2] = 0.6044910817663975;
    m_params.u_des_5[3] = -0.0840868104920891;
    m_params.x_des_6[0] = -0.900877978017443;
    m_params.x_des_6[1] = 0.608892500264739;
    m_params.x_des_6[2] = 1.8257980452695217;
    m_params.x_des_6[3] = -0.25791777529922877;
    m_params.x_des_6[4] = -1.7194699796493191;
    m_params.x_des_6[5] = -1.7690740487081298;
    m_params.x_des_6[6] = -1.6685159248097703;
    m_params.x_des_6[7] = 1.8388287490128845;
    m_params.x_des_6[8] = 0.16304334474597537;
    m_params.x_des_6[9] = 1.3498497306788897;
    m_params.x_des_6[10] = -1.3198658230514613;
    m_params.x_des_6[11] = -0.9586197090843394;
    m_params.u_des_6[0] = 0.7679100474913709;
    m_params.u_des_6[1] = 1.5822813125679343;
    m_params.u_des_6[2] = -0.6372460621593619;
    m_params.u_des_6[3] = -1.741307208038867;
    m_params.x_des_7[0] = 1.456478677642575;
    m_params.x_des_7[1] = -0.8365102166820959;
    m_params.x_des_7[2] = 0.9643296255982503;
    m_params.x_des_7[3] = -1.367865381194024;
    m_params.x_des_7[4] = 0.7798537405635035;
    m_params.x_des_7[5] = 1.3656784761245926;
    m_params.x_des_7[6] = 0.9086083149868371;
    m_params.x_des_7[7] = -0.5635699005460344;
    m_params.x_des_7[8] = 0.9067590059607915;
    m_params.x_des_7[9] = -1.4421315032701587;
    m_params.x_des_7[10] = -0.7447235390671119;
    m_params.x_des_7[11] = -0.32166897326822186;
    m_params.u_des_7[0] = 1.5088481557772684;
    m_params.u_des_7[1] = -1.385039165715428;
    m_params.u_des_7[2] = 1.5204991609972622;
    m_params.u_des_7[3] = 1.1958572768832156;
    m_params.x_des_8[0] = 1.8864971883119228;
    m_params.x_des_8[1] = -0.5291880667861584;
    m_params.x_des_8[2] = -1.1802409243688836;
    m_params.x_des_8[3] = -1.037718718661604;
    m_params.x_des_8[4] = 1.3114512056856835;
    m_params.x_des_8[5] = 1.8609125943756615;
    m_params.x_des_8[6] = 0.7952399935216938;
    m_params.x_des_8[7] = -0.07001183290468038;
    m_params.x_des_8[8] = -0.8518009412754686;
    m_params.x_des_8[9] = 1.3347515373726386;
    m_params.x_des_8[10] = 1.4887180335977037;
    m_params.x_des_8[11] = -1.6314736327976336;
    m_params.u_des_8[0] = -1.1362021159208933;
    m_params.u_des_8[1] = 1.327044361831466;
    m_params.u_des_8[2] = 1.3932155883179842;
    m_params.u_des_8[3] = -0.7413880049440107;
    m_params.x_des_9[0] = -0.8828216126125747;
    m_params.x_des_9[1] = -0.27673991192616;
    m_params.x_des_9[2] = 0.15778600105866714;
    m_params.x_des_9[3] = -1.6177327399735457;
    m_params.x_des_9[4] = 1.3476485548544606;
    m_params.x_des_9[5] = 0.13893948140528378;
    m_params.x_des_9[6] = 1.0998712601636944;
    m_params.x_des_9[7] = -1.0766549376946926;
    m_params.x_des_9[8] = 1.8611734044254629;
    m_params.x_des_9[9] = 1.0041092292735172;
    m_params.x_des_9[10] = -0.6276245424321543;
    m_params.x_des_9[11] = 1.794110587839819;
    m_params.u_des_9[0] = 0.8020471158650913;
    m_params.u_des_9[1] = 1.362244341944948;
    m_params.u_des_9[2] = -1.8180107765765245;
    m_params.u_des_9[3] = -1.7774338357932473;
    m_params.x_des_10[0] = 0.9709490941985153;
    m_params.x_des_10[1] = -0.7812542682064318;
    m_params.x_des_10[2] = 0.0671374633729811;
    m_params.x_des_10[3] = -1.374950305314906;
    m_params.x_des_10[4] = 1.9118096386279388;
    m_params.x_des_10[5] = 0.011004190697677885;
    m_params.x_des_10[6] = 1.3160043138989015;
    m_params.x_des_10[7] = -1.7038488148800144;
    m_params.x_des_10[8] = -0.08433819112864738;
    m_params.x_des_10[9] = -1.7508820783768964;
    m_params.x_des_10[10] = 1.536965724350949;
    m_params.x_des_10[11] = -0.21675928514816478;
    m_params.u_des_10[0] = -1.725800326952653;
    m_params.u_des_10[1] = -1.6940148707361717;
    m_params.u_des_10[2] = 0.15517063201268;
    m_params.u_des_10[3] = -1.697734381979077;
    m_params.x_des_11[0] = -1.264910727950229;
    m_params.x_des_11[1] = -0.2545716633339441;
    m_params.x_des_11[2] = -0.008868675926170244;
    m_params.x_des_11[3] = 0.3332476609670296;
    m_params.x_des_11[4] = 0.48205072561962936;
    m_params.x_des_11[5] = -0.5087540014293261;
    m_params.x_des_11[6] = 0.4749463319223195;
    m_params.x_des_11[7] = -1.371021366459455;
    m_params.x_des_11[8] = -0.8979660982652256;
    m_params.x_des_11[9] = 1.194873082385242;
    m_params.x_des_11[10] = -1.3876427970939353;
    m_params.x_des_11[11] = -1.106708108457053;
    m_params.Wy_final[0] = 1.2429781796939552;
    m_params.Wy_final[1] = 1.479507304823067;
    m_params.Wy_final[2] = 1.000745522041898;
    m_params.Wy_final[3] = 1.0303113605224665;
    m_params.Wy_final[4] = 1.4615481516478055;
    m_params.Wy_final[5] = 1.1625206848665193;
    m_params.Wy_final[6] = 1.6795018037732852;
    m_params.Wy_final[7] = 1.7952045871766384;
    m_params.Wy_final[8] = 1.5781633587377102;
    m_params.Wy_final[9] = 1.6947649771732056;
    m_params.Wy_final[10] = 1.3909580157338786;
    m_params.Wy_final[11] = 1.0462962029929483;
    m_params.A[0] = -0.24231386948140266;
    m_params.A[1] = -0.5120787511622411;
    m_params.A[2] = 0.3880129688013203;
    m_params.A[3] = -1.4631273212038676;
    m_params.A[4] = -1.0891484131126563;
    m_params.A[5] = 1.2591296661091191;
    m_params.A[6] = -0.9426978934391474;
    m_params.A[7] = -0.358719180371347;
    m_params.A[8] = 1.7438887059831263;
    m_params.A[9] = -0.8977901479165817;
    m_params.A[10] = -1.4188401645857445;
    m_params.A[11] = 0.8080805173258092;
    m_params.A[12] = 0.2682662017650985;
    m_params.A[13] = 0.44637534218638786;
    m_params.A[14] = -1.8318765960257055;
    m_params.A[15] = -0.3309324209710929;
    m_params.A[16] = -1.9829342633313622;
    m_params.A[17] = -1.013858124556442;
    m_params.A[18] = 0.8242247343360254;
    m_params.A[19] = -1.753837136317201;
    m_params.A[20] = -0.8212260055868805;
    m_params.A[21] = 1.9524510112487126;
    m_params.A[22] = 1.884888920907902;
    m_params.A[23] = -0.0726144452811801;
    m_params.A[24] = 0.9427735461129836;
    m_params.A[25] = 0.5306230967445558;
    m_params.A[26] = -0.1372277142250531;
    m_params.A[27] = 1.4282657305652786;
    m_params.A[28] = -1.309926991335284;
    m_params.A[29] = 1.3137276889764422;
    m_params.A[30] = -1.8317219061667278;
    m_params.A[31] = 1.4678147672511939;
    m_params.A[32] = 0.703986349872991;
    m_params.A[33] = -0.2163435603565258;
    m_params.A[34] = 0.6862809905371079;
    m_params.A[35] = -0.15852598444303245;
    m_params.A[36] = 1.1200128895143409;
    m_params.A[37] = -1.5462236645435308;
    m_params.A[38] = 0.0326297153944215;
    m_params.A[39] = 1.4859581597754916;
    m_params.A[40] = 1.71011710324809;
    m_params.A[41] = -1.1186546738067493;
    m_params.A[42] = -0.9922787897815244;
    m_params.A[43] = 1.6160498864359547;
    m_params.A[44] = -0.6179306451394861;
    m_params.A[45] = -1.7725097038051376;
    m_params.A[46] = 0.8595466884481313;
    m_params.A[47] = -0.3423245633865686;
    m_params.A[48] = 0.9412967499805762;
    m_params.A[49] = -0.09163346622652258;
    m_params.A[50] = 0.002262217745727657;
    m_params.A[51] = -0.3297523583656421;
    m_params.A[52] = -0.8380604158593941;
    m_params.A[53] = 1.6028434695494038;
    m_params.A[54] = 0.675150311940429;
    m_params.A[55] = 1.1553293733718686;
    m_params.A[56] = 1.5829581243724693;
    m_params.A[57] = -0.9992442304425597;
    m_params.A[58] = 1.6792824558896897;
    m_params.A[59] = 1.4504203490342324;
    m_params.A[60] = 0.02434104849994556;
    m_params.A[61] = 0.27160869657612263;
    m_params.A[62] = -1.5402710478528858;
    m_params.A[63] = 1.0484633622310744;
    m_params.A[64] = -1.3070999712627054;
    m_params.A[65] = 0.13534416402363814;
    m_params.A[66] = -1.4942507790851232;
    m_params.A[67] = -1.708331625671371;
    m_params.A[68] = 0.436109775042258;
    m_params.A[69] = -0.03518748153727991;
    m_params.A[70] = 0.6992397389570906;
    m_params.A[71] = 1.1634167322171374;
    m_params.A[72] = 1.9307499705822648;
    m_params.A[73] = -1.6636772756932747;
    m_params.A[74] = 0.5248484497343218;
    m_params.A[75] = 0.30789958152579144;
    m_params.A[76] = 0.602568707166812;
    m_params.A[77] = 0.17271781925751872;
    m_params.A[78] = 0.2294695501208066;
    m_params.A[79] = 1.4742185345619543;
    m_params.A[80] = -0.1919535345136989;
    m_params.A[81] = 0.13990231452144553;
    m_params.A[82] = 0.7638548150610602;
    m_params.A[83] = -1.6420200344195646;
    m_params.A[84] = -0.27229872445076087;
    m_params.A[85] = -1.5914631171820468;
    m_params.A[86] = -1.4487604283558668;
    m_params.A[87] = -1.991497766136364;
    m_params.A[88] = -1.1611742553535152;
    m_params.A[89] = -1.133450950247063;
    m_params.A[90] = 0.06497792493777155;
    m_params.A[91] = 0.28083295396097263;
    m_params.A[92] = 1.2958447220129887;
    m_params.A[93] = -0.05315524470737154;
    m_params.A[94] = 1.5658183956871667;
    m_params.A[95] = -0.41975684089933685;
    m_params.A[96] = 0.97844578833777;
    m_params.A[97] = 0.2110290496695293;
    m_params.A[98] = 0.4953003430893044;
    m_params.A[99] = -0.9184320124667495;
    m_params.A[100] = 1.750380031759156;
    m_params.A[101] = 1.0786188614315915;
    m_params.A[102] = -1.4176198837203735;
    m_params.A[103] = 0.149737479778294;
    m_params.A[104] = 1.9831452222223418;
    m_params.A[105] = -1.8037746699794734;
    m_params.A[106] = -0.7887206483295461;
    m_params.A[107] = 0.9632534854086652;
    m_params.A[108] = -1.8425542093895406;
    m_params.A[109] = 0.986684363969033;
    m_params.A[110] = 0.2936851199350441;
    m_params.A[111] = 0.9268227022482662;
    m_params.A[112] = 0.20333038350653299;
    m_params.A[113] = 1.7576139132046351;
    m_params.A[114] = -0.614393188398918;
    m_params.A[115] = 0.297877839744912;
    m_params.A[116] = -1.796880083990895;
    m_params.A[117] = 0.21373133661742738;
    m_params.A[118] = -0.32242822540825156;
    m_params.A[119] = 1.9326471511608059;
    m_params.A[120] = 1.7824292753481785;
    m_params.A[121] = -1.4468823405675986;
    m_params.A[122] = -1.8335374338761512;
    m_params.A[123] = -1.5172997317243713;
    m_params.A[124] = -1.229012129120719;
    m_params.A[125] = 0.9046719772422094;
    m_params.A[126] = 0.17591181415489432;
    m_params.A[127] = 0.13970133814112584;
    m_params.A[128] = -0.14185208214985234;
    m_params.A[129] = -1.9732231264739348;
    m_params.A[130] = -0.4301123458221334;
    m_params.A[131] = 1.9957537650387742;
    m_params.A[132] = 1.2811648216477893;
    m_params.A[133] = 0.2914428437588219;
    m_params.A[134] = -1.214148157218884;
    m_params.A[135] = 1.6818776980374155;
    m_params.A[136] = -0.30341101038214635;
    m_params.A[137] = 0.47730909231793106;
    m_params.A[138] = -1.187569373035299;
    m_params.A[139] = -0.6877370247915531;
    m_params.A[140] = -0.6201861482616171;
    m_params.A[141] = -0.4209925183921568;
    m_params.A[142] = -1.9110724537712471;
    m_params.A[143] = 0.6413882087807936;
    m_params.B[0] = -1.3200399280087032;
    m_params.B[1] = 0.41320105301312626;
    m_params.B[2] = 0.4783213861392275;
    m_params.B[3] = 0.7916189857293743;
    m_params.B[4] = -0.8322752558146558;
    m_params.B[5] = -0.8318720537426154;
    m_params.B[6] = 1.0221179076113445;
    m_params.B[7] = -0.4471032189262627;
    m_params.B[8] = -1.3901469561676985;
    m_params.B[9] = 1.6210596051208572;
    m_params.B[10] = -1.9476687601912737;
    m_params.B[11] = 1.5459376306231292;
    m_params.B[12] = -0.830972896191656;
    m_params.B[13] = -0.47269983955176276;
    m_params.B[14] = 1.913620609584223;
    m_params.B[15] = -0.25329703423935124;
    m_params.B[16] = 0.8635279149674653;
    m_params.B[17] = -0.35046893227111564;
    m_params.B[18] = 1.6541432486772365;
    m_params.B[19] = 0.8779619968413503;
    m_params.B[20] = -0.07723284625844862;
    m_params.B[21] = -1.6631134040635196;
    m_params.B[22] = -0.54546452868516;
    m_params.B[23] = -0.03757319061095998;
    m_params.B[24] = -0.864543266194465;
    m_params.B[25] = 0.13856203767859343;
    m_params.B[26] = -1.1613957272733684;
    m_params.B[27] = -0.022681697832835024;
    m_params.B[28] = 0.11202078062843634;
    m_params.B[29] = 0.6934385624164641;
    m_params.B[30] = 0.9814633803279791;
    m_params.B[31] = 0.9198949681022897;
    m_params.B[32] = -0.3035363988458051;
    m_params.B[33] = -0.1761906755724203;
    m_params.B[34] = 1.4940284058791686;
    m_params.B[35] = -0.5488483097174393;
    m_params.B[36] = 0.9521313238305416;
    m_params.B[37] = 1.9762689267600413;
    m_params.B[38] = 1.6992335341478482;
    m_params.B[39] = 0.1969474711697119;
    m_params.B[40] = -0.7795544525014559;
    m_params.B[41] = 0.4892505434034007;
    m_params.B[42] = 0.7372066729248594;
    m_params.B[43] = 0.10784901966517557;
    m_params.B[44] = -0.6340934767066218;
    m_params.B[45] = -0.17829371464242083;
    m_params.B[46] = -1.6728370279392784;
    m_params.B[47] = -0.8348711800042916;
    m_params.u_min[0] = 0.0;
    m_params.u_max[0] = 1.0;
    m_params.S[0] = 0.05;
}
