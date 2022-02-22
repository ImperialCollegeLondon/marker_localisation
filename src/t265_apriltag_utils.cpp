// based on https://github.com/IntelRealSense/librealsense/tree/master/examples/pose-apriltag
#include "t265_apriltag_utils/t265_apriltag_utils.h"

#define FORMAT_VALUE std::fixed << std::right << std::setprecision(3) << std::setw(6)

rs2_intrinsics get_rs2_int_from_camera_info(const sensor_msgs::CameraInfoConstPtr &caminf)
{
    rs2_intrinsics rsint;

    rsint.height = caminf->height;
    rsint.width = caminf->width;

    rsint.fx = caminf->K[0];
    rsint.fy = caminf->K[4];
    rsint.ppx = caminf->K[2];
    rsint.ppy = caminf->K[5];

    rsint.model = RS2_DISTORTION_FTHETA;

    const std::vector<double> d = caminf->D;
    std::copy(d.begin(), d.end(), rsint.coeffs);

    return rsint;
}

rs2_extrinsics get_rs2_ext()
{
    rs2_extrinsics rsext;

    float trans[] = {0.0, 0.0, 0.0};
    float rot[] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    std::copy(rot, rot + 9, rsext.rotation);
    std::copy(trans, trans + 3, rsext.translation);

    return rsext;
}

tf::Quaternion to_quaternion(const float (&rot)[9]){

    double w = sqrt(1.0 + rot[0] + rot[4] + rot[8]) / 2.0;
    double x = (rot[7] - rot[5]) / (4.0 * w);
    double y = (rot[2] - rot[6]) / (4.0 * w);
    double z = (rot[3] - rot[1]) / (4.0 * w);

    return tf::Quaternion(x, y, z, w); 
}


static transformation to_transform(const double R[9], const double t[3])
{
    transformation tf;
    for (int r = 0; r < 9; ++r)
    {
        tf.rotation[r] = static_cast<float>(R[r]);
    }
    for (int i = 0; i < 3; ++i)
    {
        tf.translation[i] = static_cast<float>(t[i]);
    }
    return tf;
}

static transformation to_transform(const rs2_quaternion &q, const rs2_vector &t)
{
    transformation tf;
    tf.rotation[0] = q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z;
    tf.rotation[1] = 2 * (q.x * q.y - q.w * q.z);
    tf.rotation[2] = 2 * (q.x * q.z + q.w * q.y);
    tf.rotation[3] = 2 * (q.x * q.y + q.w * q.z);
    tf.rotation[4] = q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z;
    tf.rotation[5] = 2 * (q.y * q.z - q.w * q.x);
    tf.rotation[6] = 2 * (q.x * q.z - q.w * q.y);
    tf.rotation[7] = 2 * (q.y * q.z + q.w * q.x);
    tf.rotation[8] = q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z;
    tf.translation[0] = t.x;
    tf.translation[1] = t.y;
    tf.translation[2] = t.z;
    return tf;
}

static transformation operator*(const transformation &a, const transformation &b)
{
    transformation tf;
    tf.rotation[0] = a.rotation[0] * b.rotation[0] + a.rotation[1] * b.rotation[3] + a.rotation[2] * b.rotation[6];
    tf.rotation[1] = a.rotation[0] * b.rotation[1] + a.rotation[1] * b.rotation[4] + a.rotation[2] * b.rotation[7];
    tf.rotation[2] = a.rotation[0] * b.rotation[2] + a.rotation[1] * b.rotation[5] + a.rotation[2] * b.rotation[8];
    tf.rotation[3] = a.rotation[3] * b.rotation[0] + a.rotation[4] * b.rotation[3] + a.rotation[5] * b.rotation[6];
    tf.rotation[4] = a.rotation[3] * b.rotation[1] + a.rotation[4] * b.rotation[4] + a.rotation[5] * b.rotation[7];
    tf.rotation[5] = a.rotation[3] * b.rotation[2] + a.rotation[4] * b.rotation[5] + a.rotation[5] * b.rotation[8];
    tf.rotation[6] = a.rotation[6] * b.rotation[0] + a.rotation[7] * b.rotation[3] + a.rotation[8] * b.rotation[6];
    tf.rotation[7] = a.rotation[6] * b.rotation[1] + a.rotation[7] * b.rotation[4] + a.rotation[8] * b.rotation[7];
    tf.rotation[8] = a.rotation[6] * b.rotation[2] + a.rotation[7] * b.rotation[5] + a.rotation[8] * b.rotation[8];

    tf.translation[0] = a.rotation[0] * b.translation[0] + a.rotation[1] * b.translation[1] + a.rotation[2] * b.translation[2] + a.translation[0];
    tf.translation[1] = a.rotation[3] * b.translation[0] + a.rotation[4] * b.translation[1] + a.rotation[5] * b.translation[2] + a.translation[1];
    tf.translation[2] = a.rotation[6] * b.translation[0] + a.rotation[7] * b.translation[1] + a.rotation[8] * b.translation[2] + a.translation[2];
    return tf;
}

std::string print(const transformation &tf)
{
    std::stringstream ss;
    ss << "R:";
    for (const auto &r : tf.rotation)
    {
        ss << FORMAT_VALUE << r << ",";
    }
    ss << "|t:";
    for (const auto &t : tf.translation)
    {
        ss << FORMAT_VALUE << t << ",";
    }
    return ss.str();
}

apriltag_manager::apriltag_manager(const rs2_intrinsics &_intr, const rs2_extrinsics _extr_b2f, double tagsize)
    : intr(_intr), tf_body_to_fisheye(_extr_b2f)
{
    tf = tag36h11_create();
    td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    td->quad_decimate = 1.0;
    td->quad_sigma = 0.0;
    td->nthreads = 1;
    td->debug = 0;
    td->refine_edges = 1;

    info.tagsize = tagsize;
    info.fx = info.fy = 1; // undistorted image with focal length = 1
    info.cx = info.cy = 0; // undistorted image with principal point at (0,0)
}

apriltag_manager::~apriltag_manager()
{
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
}


apriltag_manager::apriltag_array_t apriltag_manager::detect(unsigned char *gray, const rs2_pose *camera_pose) const
{
    image_u8_t img{intr.width, intr.height, intr.width, gray};

    apriltag_manager::apriltag_array_t tags;
    tags.det = std::shared_ptr<zarray_t>(apriltag_detector_detect(td, &img), apriltag_detections_destroy);
    tags.pose_in_camera.resize(zarray_size(tags.det.get()));
    tags.pose_raw.resize(tags.size());

    auto info_ = info;
    for (int t = 0, num_of_tags = (int)tags.size(); t < num_of_tags; ++t)
    {
        tags.pose_raw[t] = std::shared_ptr<apriltag_pose_t>(new apriltag_pose_t(), apriltag_pose_destroy);

        undistort(*(info_.det = tags.get(t)), intr); // recompute tag corners on an undistorted image focal length = 1
        // estimate_tag_pose(&info_, tags.pose_raw[t].get());              //(alternative) estimate tag pose in camera coordinate
        estimate_pose_for_tag_homography(&info_, tags.pose_raw[t].get()); // estimate tag pose in camera coordinate
        for (auto c : {1, 2, 4, 5, 7, 8})
        {
            tags.pose_raw[t]->R->data[c] *= -1;
        }

        tags.pose_in_camera[t] = to_transform(tags.pose_raw[t]->R->data, tags.pose_raw[t]->t->data);
    }

    if (camera_pose)
    {
        compute_tag_pose_in_world(tags, *camera_pose);
    }
    return tags;
}

void apriltag_manager::compute_tag_pose_in_world(apriltag_array_t &tags, const rs2_pose &camera_world_pose) const
{
    tags.pose_in_world.resize(tags.size());
    for (int t = 0, num_of_tags = tags.size(); t < num_of_tags; ++t)
    {
        auto tf_fisheye_to_tag = tags.pose_in_camera[t];
        auto tf_world_to_body = to_transform(camera_world_pose.rotation, camera_world_pose.translation);
        tags.pose_in_world[t] = tf_world_to_body * tf_body_to_fisheye * tf_fisheye_to_tag;
    }
}

void apriltag_manager::undistort(apriltag_detection_t &src, const rs2_intrinsics &intr)
{
    deproject(src.c, intr, src.c);

    double corr_arr[4][4];
    for (int c = 0; c < 4; ++c)
    {
        deproject(src.p[c], intr, src.p[c]);

        corr_arr[c][0] = (c == 0 || c == 3) ? -1 : 1; // tag corners in an ideal image
        corr_arr[c][1] = (c == 0 || c == 1) ? -1 : 1; // tag corners in an ideal image
        corr_arr[c][2] = src.p[c][0];                 // tag corners in undistorted image focal length = 1
        corr_arr[c][3] = src.p[c][1];                 // tag corners in undistorted image focal length = 1
    }
    if (src.H == nullptr)
    {
        src.H = matd_create(3, 3);
    }
    homography_compute2(corr_arr, src.H);
}

void apriltag_manager::deproject(double pt[2], const rs2_intrinsics &intr, const double px[2])
{
    float fpt[3], fpx[2] = {(float)px[0], (float)px[1]};
    rs2_deproject_pixel_to_point(fpt, &intr, fpx, 1.0f);
    pt[0] = fpt[0];
    pt[1] = fpt[1];
}


void homography_compute2(const double c[4][4], matd_t *H)
{
    double A[] = {
        c[0][0],
        c[0][1],
        1,
        0,
        0,
        0,
        -c[0][0] * c[0][2],
        -c[0][1] * c[0][2],
        c[0][2],
        0,
        0,
        0,
        c[0][0],
        c[0][1],
        1,
        -c[0][0] * c[0][3],
        -c[0][1] * c[0][3],
        c[0][3],
        c[1][0],
        c[1][1],
        1,
        0,
        0,
        0,
        -c[1][0] * c[1][2],
        -c[1][1] * c[1][2],
        c[1][2],
        0,
        0,
        0,
        c[1][0],
        c[1][1],
        1,
        -c[1][0] * c[1][3],
        -c[1][1] * c[1][3],
        c[1][3],
        c[2][0],
        c[2][1],
        1,
        0,
        0,
        0,
        -c[2][0] * c[2][2],
        -c[2][1] * c[2][2],
        c[2][2],
        0,
        0,
        0,
        c[2][0],
        c[2][1],
        1,
        -c[2][0] * c[2][3],
        -c[2][1] * c[2][3],
        c[2][3],
        c[3][0],
        c[3][1],
        1,
        0,
        0,
        0,
        -c[3][0] * c[3][2],
        -c[3][1] * c[3][2],
        c[3][2],
        0,
        0,
        0,
        c[3][0],
        c[3][1],
        1,
        -c[3][0] * c[3][3],
        -c[3][1] * c[3][3],
        c[3][3],
    };

    double epsilon = 1e-10;

    // Eliminate.
    for (int col = 0; col < 8; col++)
    {
        // Find best row to swap with.
        double max_val = 0;
        int max_val_idx = -1;
        for (int row = col; row < 8; row++)
        {
            double val = fabs(A[row * 9 + col]);
            if (val > max_val)
            {
                max_val = val;
                max_val_idx = row;
            }
        }

        if (max_val < epsilon)
        {
            fprintf(stderr, "WRN: Matrix is singular.\n");
        }

        // Swap to get best row.
        if (max_val_idx != col)
        {
            for (int i = col; i < 9; i++)
            {
                double tmp = A[col * 9 + i];
                A[col * 9 + i] = A[max_val_idx * 9 + i];
                A[max_val_idx * 9 + i] = tmp;
            }
        }

        // Do eliminate.
        for (int i = col + 1; i < 8; i++)
        {
            double f = A[i * 9 + col] / A[col * 9 + col];
            A[i * 9 + col] = 0;
            for (int j = col + 1; j < 9; j++)
            {
                A[i * 9 + j] -= f * A[col * 9 + j];
            }
        }
    }

    // Back solve.
    for (int col = 7; col >= 0; col--)
    {
        double sum = 0;
        for (int i = col + 1; i < 8; i++)
        {
            sum += A[col * 9 + i] * A[i * 9 + 8];
        }
        A[col * 9 + 8] = (A[col * 9 + 8] - sum) / A[col * 9 + col];
    }
    H->data[0] = A[8];
    H->data[1] = A[17];
    H->data[2] = A[26];
    H->data[3] = A[35];
    H->data[4] = A[44];
    H->data[5] = A[53];
    H->data[6] = A[62];
    H->data[7] = A[71];
    H->data[8] = 1;
}