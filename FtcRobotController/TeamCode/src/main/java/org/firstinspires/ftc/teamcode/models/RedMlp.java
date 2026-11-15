package org.firstinspires.ftc.teamcode.models;

/**
 * Recorded MLP: Limelight 6D field reading -> odometry xy.
 *
 * Direction: the Limelight pose is the INPUT, the Pinpoint odometry xy is the
 * OUTPUT. Yaw is NEVER predicted -- the encoder/gyro heading is always the
 * truth (MegaTag2 even locks to the yaw we feed it); only xy can drift.
 *
 * Inputs: ll_x, ll_y, ll_z (meters), ll_roll, ll_pitch (radians), ll_yaw
 * (radians, wrapped to +-PI), all in the RED-map field frame. Standardized
 * internally with the training-set mean/std, so callers pass raw values.
 * With MegaTag2, ll_z / ll_roll / ll_pitch are structurally 0 -- the inputs
 * are kept so the same net is ready for MT1 / full-3D data.
 *
 * Output: {odo_x, odo_y} meters, calibration-session odo frame.
 * Structure: MLP 6-6-6-6-2 tanh (tanh hidden, linear output).
 *
 * Recorded 2026-11-14 from tools/mlp_red.json
 * (train xy RMS 0.89 in vs SE(3)
 * 2.36 in on the same data).
 * Trained on 20 samples. LL inputs span 27 x 19 in of the field, odo targets span 30 x 19 in, encoder yaw 62-122 deg. Local model -- do not trust it far outside that region; retrain with field-wide samples before real use. MT2 zeroes ll_z/ll_roll/ll_pitch, so those inputs carry no signal until MT1/full-3D data is used.
 * RED-side specific: trained against the RED Hive field map (tags 30-37). Not valid for blue.
 */
public final class RedMlp {

    private RedMlp() { }

    private static final double[] IN_MEAN = {-0.34595784999999996, -0.9503504999999999, 0.0, 0.0, 0.0, 0.49063782926749333};
    private static final double[] IN_STD = {0.1887098928456786, 0.14941652350443038, 1.0, 1.0, 1.0, 0.09091129408751224};

    /** Layer weights: W[l][i][j] = input i -> output j of layer l. */
    private static final double[][][] W = {
        {
            {-0.4214030300765302, 0.006726169646250528, 0.35805638495759246, 0.24256526587714375, 0.26317090135849086, 1.056310308733266},
            {-0.30487507787768525, 0.08906870158626598, -0.4493857604159262, 0.5763311049156613, 0.1772463986886579, 0.44823597753932903},
            {1.5419598785922695e-80, -2.5505603569859605e-89, -1.1123933552555156e-65, -9.72543116251166e-37, -3.1526378355000666e-91, -1.134131354223147e-49},
            {-9.741955580611118e-90, 2.592069839102542e-75, -1.8096326965748427e-92, 8.945566811822852e-83, -1.5602290776293817e-68, 1.9004299954931276e-69},
            {5.868832757686085e-85, 3.3915987680509584e-92, -6.549281559913577e-94, -1.4997287159480405e-94, 2.3603284999048546e-85, 6.360754501566177e-87},
            {0.23936398715910767, -0.015796598498168578, -0.024186889225463346, -0.8483949240471019, -0.13454902237873753, -0.14800265205972776}
        },
        {
            {-0.27584083778088087, -0.29349319865256573, 0.2766817824338111, -0.31755000085837826, -0.5322629493726715, -0.24887874762950282},
            {-0.3057294055462237, 0.48197139046771514, -0.5916084878230565, 0.6998344703851231, -0.21369145847232052, -0.653121709820907},
            {0.4632308089712949, 0.5281198123692905, -0.1980366082910953, 0.0766953139828255, 0.09245154247112786, 0.42477293327753984},
            {0.1081564823863806, 0.40378537740541326, -0.20134442800996122, 1.136566450890464, -0.6138656353266787, 0.18015904208198605},
            {0.42741875250655625, -0.3621440523484988, 0.05871770790201811, -0.08250023539061413, 0.3227513035389299, -0.042850029568980534},
            {-0.5756873918365281, -0.05217174391689647, 0.3170383193544479, -0.022631863827581727, 0.1456850660521861, -0.1559611271401041}
        },
        {
            {0.8920980182248169, 0.5358858421532086, 0.17965655985762743, 0.28428921411507485, -0.09529355333308832, 0.2396871979110485},
            {-0.6154105791345557, -0.9063926088941906, 0.25606819888488463, -0.17737480473765632, 0.11843386578112378, -0.12893052828503987},
            {-0.21746890009794223, 1.1276663023231217, 0.4863135082878902, 0.07488694359448897, 0.08834328606514043, 0.7245246022984916},
            {0.2565626884385446, 0.609748202851549, 0.11781430022690043, -0.2560960295689275, 0.23688931105863326, 0.24109625665318216},
            {0.2789902257263501, 0.482317036545999, 0.429190233306228, -0.14437301102779887, 0.7858926730315262, 0.06251138692259822},
            {0.40594893960214673, -0.37732892344646046, 0.5430397377382596, 0.37771981856362963, -0.14414966717616712, 0.5159545614811741}
        },
        {
            {0.5404684825791866, 0.46660478884002554},
            {0.6269259338144012, -0.19018674795590806},
            {0.1981883800495013, -0.7415641690681287},
            {0.2961381668320706, -0.3982737765065848},
            {0.627105164095569, 0.7057643140705153},
            {0.5535925136046626, -0.8243629514612628}
        }
    };
    private static final double[][] B = {
        {-0.13926014207085566, 0.1751599782842314, 0.2858471874815547, 0.03984812358215916, -0.11493328725077002, -0.12474060256146702},
        {-0.186304273960938, 0.21770357479943458, -0.12336044497630966, -0.056480588988589674, -0.04403942281925493, -0.16545980984436226},
        {-0.2133040637355302, -0.15450933688378882, -0.10871617377453405, -0.14829478955190423, -0.1614751572927504, -0.13201728536502438},
        {-0.15724105480202646, 0.08206244158913978}
    };

    /**
     * @param llX     Limelight field x, meters (RED map)
     * @param llY     Limelight field y, meters
     * @param llZ     Limelight field z, meters (0 with MegaTag2)
     * @param llRoll  roll, radians (0 with MegaTag2)
     * @param llPitch pitch, radians (0 with MegaTag2)
     * @param llYaw   yaw, radians, wrapped to +-PI
     * @return {odo_x, odo_y} in meters, calibration odo frame -- take yaw
     *         from the encoders, never from a model
     */
    public static double[] predict(double llX, double llY, double llZ,
                                   double llRoll, double llPitch, double llYaw) {
        double[] v = {
                (llX - IN_MEAN[0]) / IN_STD[0],
                (llY - IN_MEAN[1]) / IN_STD[1],
                (llZ - IN_MEAN[2]) / IN_STD[2],
                (llRoll - IN_MEAN[3]) / IN_STD[3],
                (llPitch - IN_MEAN[4]) / IN_STD[4],
                (llYaw / Math.PI - IN_MEAN[5]) / IN_STD[5]
        };
        for (int l = 0; l < W.length; l++) {
            boolean last = (l == W.length - 1);
            double[] next = new double[B[l].length];
            for (int j = 0; j < next.length; j++) {
                double s = B[l][j];
                for (int i = 0; i < v.length; i++) s += v[i] * W[l][i][j];
                next[j] = last ? s : Math.tanh(s);
            }
            v = next;
        }
        return v;
    }
}
