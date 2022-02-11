#include <iostream>
#include </home/bot/eigen/Eigen/Core>
#include </home/bot/eigen/Eigen/SVD>
#include </home/bot/eigen/Eigen/Dense>
#include <vector>

// using Eigen::MatrixXf;
using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;
;
int QuaterniondToMatrix(Eigen::Quaterniond &qn, Eigen::Matrix3d &mx)
{
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = qn.matrix();
    mx = rotation_matrix;
    return 0;
}

double optimize_T_wc(std::vector<Eigen::Matrix4d> &Trans_we, std::vector<Eigen::Matrix4d> &Trans_cp, Eigen::Matrix4d &Trans_ep, Eigen::Matrix4d &Trans_wc)
{
    double residual = 0;
    std::vector<Eigen::Matrix4d> Trans_wp;
    for (int i = 0; i < Trans_we.size(); i++)
    {
        Eigen::Matrix4d wp = Trans_we[i] * Trans_ep;
        Trans_wp.push_back(wp);
        /* code */
    }

    Eigen::Vector3d t_wp;
    Eigen::Vector3d t_cp;
    // least squares solution parameters solve Trans_wc

    // Trans_wc =;
    Eigen::Matrix3d R_wc;
    Eigen::Vector3d t_wc;
    // Eigen::Vector3d delta = R_wc.dot(t_cp) + t_wc - t_wp;
    // residual = np.sum(np.linalg.norm(delta, axis=0)) / t_wp.shape[1]

    return residual;
}

double optimize_T_ep_linear(std::vector<Eigen::Matrix4d> &Trans_we, std::vector<Eigen::Matrix4d> &Trans_cp, Eigen::Matrix4d &Trans_ep, Eigen::Matrix4d &Trans_wc)
{
    std::vector<Eigen::Matrix4d> Trans_wp;
    for (int i = 0; i < Trans_we.size(); i++)
    {
        Eigen::Matrix4d wp = Trans_we[i] * Trans_ep;
        Trans_wp.push_back(wp);
        /* code */
    }
}

double pp_optimize(std::vector<Eigen::Matrix4d> &Trans_we, std::vector<Eigen::Matrix4d> &Trans_cp, Eigen::Matrix4d &Trans_ep, Eigen::Matrix4d &Trans_wc, int max_iteration = 1000, double min_residual_change = 1e-6)
{
    double last_residual = 1e9;
    bool converged = false;

    Trans_ep = Eigen::Matrix4d::Identity();
    Trans_wc = Eigen::Matrix4d::Identity();

    for (int i = 0; i < max_iteration; i++)
    {
        /* code */
        double residual = optimize_T_wc(Trans_we, Trans_cp, Trans_ep, Trans_wc);
        optimize_T_ep_linear(Trans_we, Trans_cp, Trans_ep, Trans_wc);

        printf("Residual: %f", residual);

        if (abs(last_residual - residual) < min_residual_change)
        {
            converged = true;
            last_residual = residual;
            break;
        }

        last_residual = residual;
    }

    if (converged)
    {
        printf("optimize finished. min_residual_change condition satisfied.");
    }
    else
    {
        printf("optimize finished. max_iteration condition satisfied.");
    }

    std::cout << "Trans_wc:" << std::endl;
    std::cout << Trans_wc << std::endl;
    std::cout << "Trans_ep:" << std::endl;
    std::cout << Trans_wc << std::endl;

    return last_residual;
}

int main()
{
    std::vector<Eigen::Matrix4d> trans_we;
    std::vector<Eigen::Matrix4d> trans_cp;

    {
        {
            Eigen::Quaterniond quaternion(4.0294304850182956e-17, 4.610605657931685e-17, 0.7529690456287916, 0.6580559370789589);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(-0.053414902, 1.7037131349999999, 0.919004333);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_we.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        {
            Eigen::Quaterniond quaternion(-0.14688575362251538, 0.7043123203164854, 0.6793338457421606, -0.14447926101112452);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(-0.14521923661231995, -0.3423176407814026, 1.040000081062317);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_cp.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        Matrix4d abc = trans_cp[trans_cp.size() - 1] * trans_we[trans_we.size() - 1];
        std::cout << abc << std::endl;
    }
    std::cout << "------------------------------------" << std::endl;

    {
        {
            Eigen::Quaterniond quaternion(5.770273703516176e-17, 2.048886515416644e-17, -0.33460856090803626, -0.9423572098557176);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(-0.10885489699999999, 1.6498936770000001, 0.8606843260000001);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_we.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        {
            Eigen::Quaterniond quaternion(-0.2170988200688304, 0.27857612303315593, 0.9329194030284309, -0.07017715763418812);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(-0.12358926981687546, -0.05413820967078209, 1.0850000381469727);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_cp.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        Matrix4d abc = trans_cp[trans_cp.size() - 1] * trans_we[trans_we.size() - 1];
        std::cout << abc << std::endl;
    }
    std::cout << "------------------------------------" << std::endl;

    {
        {
            Eigen::Quaterniond quaternion(5.770273703516176e-17, 2.048886515416644e-17, -0.33460856090803626, -0.9423572098557176);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(-0.10885489699999999, 1.835473145, 1.037623779);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_we.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        {
            Eigen::Quaterniond quaternion(-0.20990537425668843, 0.28121422586037575, 0.9333024451142063, -0.07618949386728414);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(-0.029656002297997475, -0.053343772888183594, 0.8500000238418579);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_cp.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        Matrix4d abc = trans_cp[trans_cp.size() - 1] * trans_we[trans_we.size() - 1];
        std::cout << abc << std::endl;
    }
    std::cout << "------------------------------------" << std::endl;

    {
        {
            Eigen::Quaterniond quaternion(5.770273703516176e-17, 2.048886515416644e-17, 0.33460856090803626, 0.9423572098557176);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(-0.10885489699999999, 1.835473145, 0.897314026);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_we.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        {
            Eigen::Quaterniond quaternion(-0.2154147076218061, 0.279726165500852, 0.9328792847185687, -0.07131631101910187);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(0.028721099719405174, -0.04869943857192993, 0.9760000705718994);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_cp.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        Matrix4d abc = trans_cp[trans_cp.size() - 1] * trans_we[trans_we.size() - 1];
        std::cout << abc << std::endl;
    }
    std::cout << "------------------------------------" << std::endl;

    {
        {
            Eigen::Quaterniond quaternion(2.2637698657668766e-17, 5.689405993721336e-17, 0.9291505106096748, 0.3697016751838981);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(0.184094635, 1.835473145, 0.897314026);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_we.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        {
            Eigen::Quaterniond quaternion(-0.07828282205888248, 0.8917561025180899, 0.40553993527112164, -0.1848789179234281);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(-0.2082851380109787, -0.22660695016384125, 1.090000033378601);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_cp.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        Matrix4d abc = trans_cp[trans_cp.size() - 1] * trans_we[trans_we.size() - 1];
        std::cout << abc << std::endl;
    }
    std::cout << "------------------------------------" << std::endl;

    {
        {
            Eigen::Quaterniond quaternion(2.2637698657668766e-17, 5.689405993721336e-17, 0.9291505106096748, 0.3697016751838981);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(0.43960434, 1.835473145, 0.897314026);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_we.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        {
            Eigen::Quaterniond quaternion(-0.07976139475696001, 0.891202957735556, 0.4056684281987657, -0.18662404559049706);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(-0.22189830243587494, 0.02766597829759121, 1.0880000591278076);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_cp.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        Matrix4d abc = trans_cp[trans_cp.size() - 1] * trans_we[trans_we.size() - 1];
        std::cout << abc << std::endl;
    }
    std::cout << "------------------------------------" << std::endl;

    {
        {
            Eigen::Quaterniond quaternion(8.412466858042346e-18, 6.065170944018795e-17, 0.9905175840481685, 0.13738600980951296);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(0.43960434, 1.835473145, 0.897314026);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_we.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        {
            Eigen::Quaterniond quaternion(-0.022318884191351987, 0.964253496238629, 0.1833930268692599, -0.189958048252145);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(-0.37597015500068665, 0.029395103454589844, 1.156000018119812);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_cp.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        Matrix4d abc = trans_cp[trans_cp.size() - 1] * trans_we[trans_we.size() - 1];
        std::cout << abc << std::endl;
    }
    std::cout << "------------------------------------" << std::endl;

    {
        {
            Eigen::Quaterniond quaternion(8.412466858042346e-18, 6.065170944018795e-17, 0.9905175840481685, 0.13738600980951296);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(0.295784943, 1.970113159, 0.897314026);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_we.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        {
            Eigen::Quaterniond quaternion(-0.023872735917628438, 0.9641610590673076, 0.1827139054119579, -0.19089047495127986);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(-0.24702925980091095, -0.10762595385313034, 1.1019999980926514);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_cp.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        Matrix4d abc = trans_cp[trans_cp.size() - 1] * trans_we[trans_we.size() - 1];
        std::cout << abc << std::endl;
    }
    std::cout << "------------------------------------" << std::endl;

    {
        {
            Eigen::Quaterniond quaternion(8.412466858042346e-18, 6.065170944018795e-17, 0.9905175840481685, 0.13738600980951296);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(0.43960434, 1.641883423, 0.897314026);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_we.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        {
            Eigen::Quaterniond quaternion(-0.029520202304255543, 0.9625401601172447, 0.1863249934191143, -0.19475110948272314);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(-0.5505127310752869, 0.020107947289943695, 1.2380000352859497);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_cp.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        Matrix4d abc = trans_cp[trans_cp.size() - 1] * trans_we[trans_we.size() - 1];
        std::cout << abc << std::endl;
    }
    std::cout << "------------------------------------" << std::endl;

    {
        {
            Eigen::Quaterniond quaternion(7.499068640335478e-18, 6.077140302956795e-17, 0.9924723287053765, 0.12246908489136006);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(0.295784943, 1.970113159, 0.5610740359999999);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_we.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        {
            Eigen::Quaterniond quaternion(-0.028900204822305216, 0.9662845174171332, 0.17003410262338586, -0.19117377829144586);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(-0.11697462946176529, -0.09400096535682678, 1.415000081062317);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_cp.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        Matrix4d abc = trans_cp[trans_cp.size() - 1] * trans_we[trans_we.size() - 1];
        std::cout << abc << std::endl;
    }
    std::cout << "------------------------------------" << std::endl;

    {
        {
            Eigen::Quaterniond quaternion(2.6663794132159847e-17, 5.512206036726532e-17, 0.9002115614990942, 0.43545280403662867);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(0.295784943, 1.970113159, 0.5610740359999999);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_we.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        {
            Eigen::Quaterniond quaternion(-0.09782049811293178, 0.8592314254346413, 0.46874453362204754, -0.18008628458031306);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(0.0923483595252037, -0.08296525478363037, 1.3220000267028809);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_cp.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        Matrix4d abc = trans_cp[trans_cp.size() - 1] * trans_we[trans_we.size() - 1];
        std::cout << abc << std::endl;
    }
    std::cout << "------------------------------------" << std::endl;

    {
        {
            Eigen::Quaterniond quaternion(2.6663794132159847e-17, 5.512206036726532e-17, 0.9002115614990942, 0.43545280403662867);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(0.61897467, 1.970113159, 0.5610740359999999);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_we.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        {
            Eigen::Quaterniond quaternion(-0.09438750144662622, 0.8597996706367858, 0.46701019475698724, -0.18367635649854022);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(0.07517202943563461, 0.23951776325702667, 1.319000005722046);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_cp.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        Matrix4d abc = trans_cp[trans_cp.size() - 1] * trans_we[trans_we.size() - 1];
        std::cout << abc << std::endl;
    }
    std::cout << "------------------------------------" << std::endl;

    {
        {
            Eigen::Quaterniond quaternion(2.6663794132159847e-17, 5.512206036726532e-17, 0.9002115614990942, 0.43545280403662867);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(0.29722491500000003, 1.970113159, 0.5610740359999999);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_we.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        {
            Eigen::Quaterniond quaternion(-0.09949407634621274, 0.8576027406472568, 0.4719463028571725, -0.17856414876949442);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(0.0923483595252037, -0.08296525478363037, 1.3220000267028809);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_cp.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        Matrix4d abc = trans_cp[trans_cp.size() - 1] * trans_we[trans_we.size() - 1];
        std::cout << abc << std::endl;
    }
    std::cout << "------------------------------------" << std::endl;

    {
        {
            Eigen::Quaterniond quaternion(4.0294304850182956e-17, 4.610605657931685e-17, -0.7529690456287916, -0.6580559370789589);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(-0.001665188, 1.934022827, 0.919004333);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_we.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        {
            Eigen::Quaterniond quaternion(-0.148059998948307, 0.704120052752282, 0.6780993383824226, -0.14991489355254745);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(0.06086769700050354, -0.2818315327167511, 0.9460000395774841);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_cp.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        Matrix4d abc = trans_cp[trans_cp.size() - 1] * trans_we[trans_we.size() - 1];
        std::cout << abc << std::endl;
    }
    std::cout << "------------------------------------" << std::endl;

    {
        {
            Eigen::Quaterniond quaternion(4.9723266525966786e-17, 3.573508391822125e-17, 0.5835982087749938, 0.8120425670582908);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(-0.001665188, 1.934022827, 0.919004333);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_we.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        {
            Eigen::Quaterniond quaternion(-0.1832915813308695, 0.5284883622959562, 0.8201634071477518, -0.120150874764848);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(0.12095344066619873, -0.13460353016853333, 0.9140000343322754);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_cp.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        Matrix4d abc = trans_cp[trans_cp.size() - 1] * trans_we[trans_we.size() - 1];
        std::cout << abc << std::endl;
    }
    std::cout << "------------------------------------" << std::endl;

    {
        {
            Eigen::Quaterniond quaternion(4.9723266525966786e-17, 3.573508391822125e-17, 0.5835982087749938, 0.8120425670582908);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(-0.001665188, 1.934022827, 1.085234009);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_we.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        {
            Eigen::Quaterniond quaternion(-0.18320389397620124, 0.5317586794135579, 0.8178475186739486, -0.12163254621675472);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(0.05042900890111923, -0.1402176469564438, 0.7620000243186951);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_cp.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        Matrix4d abc = trans_cp[trans_cp.size() - 1] * trans_we[trans_we.size() - 1];
        std::cout << abc << std::endl;
    }
    std::cout << "------------------------------------" << std::endl;

    {
        {
            Eigen::Quaterniond quaternion(4.9723266525966786e-17, 3.573508391822125e-17, 0.5835982087749938, 0.8120425670582908);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(0.143684525, 1.934022827, 0.8606843260000001);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_we.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        {
            Eigen::Quaterniond quaternion(-0.1810613245306715, 0.5323508800363477, 0.8188072302897902, -0.11564625765189197);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(0.13656902313232422, 0.012128004804253578, 0.9650000333786011);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_cp.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        Matrix4d abc = trans_cp[trans_cp.size() - 1] * trans_we[trans_we.size() - 1];
        std::cout << abc << std::endl;
    }
    std::cout << "------------------------------------" << std::endl;

    {
        {
            Eigen::Quaterniond quaternion(5.261626669000498e-17, 3.131976909654534e-17, 0.5114906456024935, 0.859288845186032);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(0.143684525, 1.780033203, 0.8606843260000001);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_we.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        {
            Eigen::Quaterniond quaternion(-0.19726020997467383, 0.458271837550642, 0.85975669866088, -0.10905847778348296);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(0.0019096017349511385, 0.06560975313186646, 1.0250000953674316);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_cp.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        Matrix4d abc = trans_cp[trans_cp.size() - 1] * trans_we[trans_we.size() - 1];
        std::cout << abc << std::endl;
    }
    std::cout << "------------------------------------" << std::endl;

    {
        {
            Eigen::Quaterniond quaternion(5.261626669000498e-17, 3.131976909654534e-17, -0.5114906456024935, -0.859288845186032);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(-0.10885489699999999, 1.780033203, 0.8606843260000001);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_we.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        {
            Eigen::Quaterniond quaternion(-0.19573487249375, 0.45640890745276597, 0.8617413240841205, -0.10382995354604976);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(0.01704435795545578, -0.18556807935237885, 1.0290000438690186);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_cp.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        Matrix4d abc = trans_cp[trans_cp.size() - 1] * trans_we[trans_we.size() - 1];
        std::cout << abc << std::endl;
    }
    std::cout << "------------------------------------" << std::endl;

    {
        {
            Eigen::Quaterniond quaternion(5.770273703516176e-17, 2.048886515416644e-17, -0.33460856090803626, -0.9423572098557176);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(-0.10885489699999999, 1.780033203, 0.8606843260000001);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_we.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        {
            Eigen::Quaterniond quaternion(-0.21115858808945956, 0.2789460854261307, 0.933873523504676, -0.07403630324688673);
            Eigen::Matrix3d rot;
            QuaterniondToMatrix(quaternion, rot);
            Eigen::Vector3d tr(-0.005668547935783863, -0.049645740538835526, 1.0329999923706055);
            Matrix4d trans;                // transformation matrix
            trans.setIdentity();           // set to identity
            trans.block<3, 3>(0, 0) = rot; // first 3x3 block set to rotation matrix
            trans.block<3, 1>(0, 3) = tr;  // fourth column set to translation vector
            trans_cp.push_back(trans);
            std::cout << trans << std::endl;
            std::cout << std::endl;
        }
        Matrix4d abc = trans_cp[trans_cp.size() - 1] * trans_we[trans_we.size() - 1];
        std::cout << abc << std::endl;
    }
    std::cout << "------------------------------------" << std::endl;
    Eigen::Matrix4d trans_wc, trans_ep;
    //pp_optimize(trans_we, trans_cp, trans_ep, trans_wc);

    Matrix3f A;
    // A(0, 0) = 1, A(0, 1) = 0, A(0, 2) = 1;
    // A(1, 0) = 0, A(1, 1) = 1, A(1, 2) = 1;
    // A(2, 0) = 0, A(2, 1) = 0, A(2, 2) = 0;
    // JacobiSVD<Eigen::MatrixXf> svd(A, ComputeThinU | ComputeThinV);
    // Matrix3f V = svd.matrixV(), U = svd.matrixU();
    // Matrix3f  S = U.inverse() * A * V.transpose().inverse(); // S = U^-1 * A * VT * -1
    // std::cout << "A :\n" << A << std::endl;
    // std::cout << "U :\n" << U << std::endl;
    // std::cout << "S :\n" << S << std::endl;
    // std::cout << "V :\n" << V << std::endl;
    // std::cout << "U * S * VT :\n" << U * S * V.transpose() << std::endl;
    // system("pause");
    return 0;
}
