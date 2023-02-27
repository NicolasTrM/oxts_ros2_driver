// Copyright 2021 Oxford Technical Solutions Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "oxts_ins/convert.hpp"

namespace oxts_ins {
#define TWO_DIM_ONLY 1

void OxtsIns::ncomCallbackRegular(const oxts_msgs::msg::Ncom::SharedPtr msg) {
  // Add data to decoder
  if (NComNewChars(this->nrx, msg->raw_packet.data(), NCOM_PACKET_LENGTH) ==
      COM_NEW_UPDATE) {
    double current_time = rclcpp::Time(msg->header.stamp).seconds();
    int sec_idx = round((current_time - floor(current_time)) * this->ncom_rate);

    // Publish IMU message if being subscribed to and enabled in config
    if (this->pub_imu_flag)
      this->imu(msg->header);
    if (this->pub_tf_flag)
      this->tf(msg->header);
    if (this->pubStringInterval && (sec_idx % this->pubStringInterval == 0))
      this->string();
    if (this->pubNavSatRefInterval &&
        (sec_idx % this->pubNavSatRefInterval == 0))
      this->nav_sat_ref(msg->header);
    if (this->pubEcefPosInterval && (sec_idx % this->pubEcefPosInterval == 0))
      this->ecef_pos(msg->header);
    if (this->pubNavSatFixInterval &&
        (sec_idx % this->pubNavSatFixInterval == 0))
      this->nav_sat_fix(msg->header);
    if (this->pubVelocityInterval && (sec_idx % this->pubVelocityInterval == 0))
      this->velocity(msg->header);
    if (this->pubOdometryInterval && (sec_idx % this->pubOdometryInterval == 0))
      this->odometry(msg->header);
    if (this->pubPathInterval && (sec_idx % this->pubPathInterval == 0))
      this->path(msg->header);
    if (this->pubTimeReferenceInterval &&
        (sec_idx % this->pubTimeReferenceInterval == 0))
      this->time_reference(msg->header);
    if (this->pubLeverArmInterval && (sec_idx % this->pubLeverArmInterval == 0))
      this->lever_arm_gap(msg->header);
    if (this->pubIMUBiasInterval && (sec_idx % this->pubIMUBiasInterval == 0))
      this->imu_bias(msg->header);
  }
}

void OxtsIns::string() {
  auto msgString = RosNComWrapper::string(this->nrx);
  pubString_->publish(msgString);

  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msgString.data.c_str());
}

void OxtsIns::nav_sat_fix(std_msgs::msg::Header header) {
  header.frame_id = "navsat_link";
  auto msg = RosNComWrapper::nav_sat_fix(this->nrx, header);
  pubNavSatFix_->publish(msg);
}

void OxtsIns::nav_sat_ref(std_msgs::msg::Header header) {
  // Set the LRF if - we haven't set it before (unless using NCOM LRF)
  this->getLrf();
  if (this->lrf_valid) {
    header.frame_id = "navsat_link";
    auto msg = RosNComWrapper::nav_sat_ref(this->lrf, header);
    pubNavSatRef_->publish(msg);
  }
}

void OxtsIns::lever_arm_gap(std_msgs::msg::Header header) {
  header.frame_id = "oxts_link";
  auto msg = RosNComWrapper::lever_arm_gap(this->nrx, header);
  pubLeverArm_->publish(msg);
}

void OxtsIns::imu_bias(std_msgs::msg::Header header) {
  header.frame_id = "oxts_link";
  auto msg = RosNComWrapper::imu_bias(this->nrx, header);
  pubIMUBias_->publish(msg);
}

void OxtsIns::ecef_pos(std_msgs::msg::Header header) {
  header.frame_id = "oxts_link";
  auto msg = RosNComWrapper::ecef_pos(this->nrx, header);
  pubEcefPos_->publish(msg);
}

void OxtsIns::imu(std_msgs::msg::Header header) {
  header.frame_id = "imu_link";
  auto msg = RosNComWrapper::imu(this->nrx, header);
  pubImu_->publish(msg);
}

void OxtsIns::tf(const std_msgs::msg::Header &header) {
  // Set the LRF if - we haven't set it before (unless using NCOM LRF)
  this->getLrf();
  Lrf origine = this->lrf ;
  #if TWO_DIM_ONLY 
  //origine = RosNComWrapper::getmyOriginLrf();
  #endif
  if (this->lrf_valid) {
    auto odometry = RosNComWrapper::odometry(this->nrx, header, origine  );
    geometry_msgs::msg::TransformStamped tf_oxts;
    tf_oxts.header = header;
    tf_oxts.header.frame_id = this->pub_odometry_frame_id;
    tf_oxts.child_frame_id = "oxts_link";
    tf_oxts.transform.translation.x = odometry.pose.pose.position.x;
    tf_oxts.transform.translation.y = odometry.pose.pose.position.y;
    tf_oxts.transform.translation.z = odometry.pose.pose.position.z;

    tf_oxts.transform.rotation = odometry.pose.pose.orientation;


 #if TWO_DIM_ONLY 

      tf_oxts.transform.translation.z = 0 ;
      
      tf_oxts.transform.rotation.x = 0;
      tf_oxts.transform.rotation.y = 0;
      double norm = sqrt ( pow( tf_oxts.transform.rotation.z  , 2) + pow( tf_oxts.transform.rotation.w , 2)  ) ;
      tf_oxts.transform.rotation.z /= norm;
      tf_oxts.transform.rotation.w /= norm;

#endif

    tf_broadcaster_->sendTransform(tf_oxts);

 

    geometry_msgs::msg::TransformStamped map_odom;
    map_odom.header = header;
    map_odom.header.frame_id = this->pub_odometry_frame_id;
    map_odom.child_frame_id = "odom";
    map_odom.transform.translation.x = 0.0 ; // nvsp_wFR.x();
    map_odom.transform.translation.y = 0.0 ; // nvsp_wFR.y();
    map_odom.transform.translation.z = 0.0 ; // nvsp_wFR.z();
    map_odom.transform.rotation.x = 0.0 ; //  nvsp_wFR.x();
    map_odom.transform.rotation.y = 0.0 ; //  nvsp_wFR.y();
    map_odom.transform.rotation.z = 0.0 ; //  nvsp_wFR.z();
    map_odom.transform.rotation.w = 1.0 ; // nvsp_wFR.w();
    tf_broadcaster_->sendTransform(map_odom);





    auto vat = RosNComWrapper::getVat(this->nrx);
    auto nsp = RosNComWrapper::getNsp(this->nrx);

    if (this->nrx->mIsNoSlipLeverArmXValid) {
      geometry_msgs::msg::TransformStamped tf_vat;
      tf_vat.header = header;
      tf_vat.header.frame_id = "oxts_link";
      tf_vat.child_frame_id = "rear_axle_link";
      tf_vat.transform.translation.x = nsp.x();
      tf_vat.transform.translation.y = nsp.y();
      tf_vat.transform.translation.z = nsp.z();
      tf_vat.transform.rotation.x = vat.x();
      tf_vat.transform.rotation.y = vat.y();
      tf_vat.transform.rotation.z = vat.z();
      tf_vat.transform.rotation.w = vat.w();

 #if TWO_DIM_ONLY 

      tf_vat.transform.translation.x = -1.1;
      tf_vat.transform.translation.y = 0 ;
      tf_vat.transform.translation.z = 0 ;
      
      tf_vat.transform.rotation.x = 0;
      tf_vat.transform.rotation.y = 0;
      tf_vat.transform.rotation.z = 0.0;
      tf_vat.transform.rotation.w = 1.0;

#endif
      tf_broadcaster_->sendTransform(tf_vat);

      if (true){  
          geometry_msgs::msg::TransformStamped tf_rear_wheel_right;
          tf_rear_wheel_right.header = header;
          tf_rear_wheel_right.header.frame_id = "rear_axle_link";
          tf_rear_wheel_right.child_frame_id = "rear_right_wheel_link";
          tf_rear_wheel_right.transform.translation.x = 0.0 ; // nvsp_wFR.x();
          tf_rear_wheel_right.transform.translation.y = 0.99 ; // nvsp_wFR.y();
          tf_rear_wheel_right.transform.translation.z = -0.0 ; // nvsp_wFR.z();
          tf_rear_wheel_right.transform.rotation.x = 0.0 ; //  nvsp_wFR.x();
          tf_rear_wheel_right.transform.rotation.y = 0.0 ; //  nvsp_wFR.y();
          tf_rear_wheel_right.transform.rotation.z = 0.0 ; //  nvsp_wFR.z();
          tf_rear_wheel_right.transform.rotation.w = 1.0 ; // nvsp_wFR.w();
          tf_broadcaster_->sendTransform(tf_rear_wheel_right);


          geometry_msgs::msg::TransformStamped tf_rear_wheel_left;
          tf_rear_wheel_left.header = header;
          tf_rear_wheel_left.header.frame_id = "rear_axle_link";
          tf_rear_wheel_left.child_frame_id = "rear_left_wheel_link";
          tf_rear_wheel_left.transform.translation.x = 0.0 ; // nvsp_wFR.x();
          tf_rear_wheel_left.transform.translation.y = -0.99 ; // nvsp_wFR.y();
          tf_rear_wheel_left.transform.translation.z = -0.0 ; // nvsp_wFR.z();
          tf_rear_wheel_left.transform.rotation.x = 0.0 ; //  nvsp_wFR.x();
          tf_rear_wheel_left.transform.rotation.y = 0.0 ; //  nvsp_wFR.y();
          tf_rear_wheel_left.transform.rotation.z = 0.0 ; //  nvsp_wFR.z();
          tf_rear_wheel_left.transform.rotation.w = 1.0 ; // nvsp_wFR.w();
          tf_broadcaster_->sendTransform(tf_rear_wheel_left);
      }

      if (true) // if vertical slip lever arm is valid
      {
        // auto nvsp    = RosNComWrapper::getNvsp(this->nrx);
        /** \todo Make this real */
        // The transform to create the front axle pose is spoofed with a hardcoded offset, for now.
        auto nvsp = nsp;
        nvsp += tf2::quatRotate(vat, tf2::Vector3(2.6, 0, 0));

        geometry_msgs::msg::TransformStamped tf_front_axle;
        tf_front_axle.header = header;
        tf_front_axle.header.frame_id = "oxts_link";
        tf_front_axle.child_frame_id = "front_axle_link";
        tf_front_axle.transform.translation.x = nvsp.x();
        tf_front_axle.transform.translation.y = nvsp.y();
        tf_front_axle.transform.translation.z = nvsp.z();
        tf_front_axle.transform.rotation.x = vat.x();
        tf_front_axle.transform.rotation.y = vat.y();
        tf_front_axle.transform.rotation.z = vat.z();
        tf_front_axle.transform.rotation.w = vat.w();


 #if TWO_DIM_ONLY 

        tf_front_axle.transform.translation.x = 2.5 ;
        tf_front_axle.transform.translation.y = 0.0 ; 
        tf_front_axle.transform.translation.z = 0 ;
        
        tf_front_axle.transform.rotation.x = 0;
        tf_front_axle.transform.rotation.y = 0; 
        tf_front_axle.transform.rotation.z = 0 ;
        tf_front_axle.transform.rotation.w = 1 ;

#endif


        tf_broadcaster_->sendTransform(tf_front_axle);

        // Aditionnal transforme wheels
        if(true)
        {
          auto nvsp_wFR = nsp;
          nvsp_wFR += tf2::quatRotate(vat, tf2::Vector3(2.6, -1.73/2, 0));

          geometry_msgs::msg::TransformStamped tf_front_wheel_right;
          tf_front_wheel_right.header = header;
          tf_front_wheel_right.header.frame_id = "front_axle_link";
          tf_front_wheel_right.child_frame_id = "caster_front_right_link";
          tf_front_wheel_right.transform.translation.x = 0.0 ; // nvsp_wFR.x();
          tf_front_wheel_right.transform.translation.y = -0.87 ; // nvsp_wFR.y();
          tf_front_wheel_right.transform.translation.z = -0.0 ; // nvsp_wFR.z();
          tf_front_wheel_right.transform.rotation.x = 0.0 ; //  nvsp_wFR.x();
          tf_front_wheel_right.transform.rotation.y = 0.0 ; //  nvsp_wFR.y();
          tf_front_wheel_right.transform.rotation.z = 0.0 ; //  nvsp_wFR.z();
          tf_front_wheel_right.transform.rotation.w = 1.0 ; // nvsp_wFR.w();

 #if TWO_DIM_ONLY 

          tf_front_wheel_right.transform.translation.z = 0 ;
           

#endif
          tf_broadcaster_->sendTransform(tf_front_wheel_right);


           //OxTs_Tr_Point01
          tf_front_wheel_right.header.frame_id = "front_axle_link";
          tf_front_wheel_right.child_frame_id = "OxTs_Tr_Point01";
          tf_front_wheel_right.transform.translation.y =  0.87 ; // nvsp_wFR.y();
          tf_broadcaster_->sendTransform(tf_front_wheel_right);

        }
      }
    }
  }
}

void OxtsIns::velocity(std_msgs::msg::Header header) {
  header.frame_id = "oxts_link";
  auto msg = RosNComWrapper::velocity(this->nrx, header);
  pubVelocity_->publish(msg);
}

void OxtsIns::odometry(std_msgs::msg::Header header) {
  header.frame_id = this->pub_odometry_frame_id;
  // Set the LRF if - we haven't set it before (unless using NCOM LRF)
  this->getLrf();
  if (this->lrf_valid) {
    auto msg = RosNComWrapper::odometry(this->nrx, header, this->lrf);
    if (this->pubPathInterval) {
      auto new_pose_stamped = geometry_msgs::msg::PoseStamped();
      new_pose_stamped.header = msg.header;
      new_pose_stamped.pose = msg.pose.pose;

 
      this->past_poses.push_back(new_pose_stamped);
    }
    pubOdometry_->publish(msg);
  }
}

void OxtsIns::path(std_msgs::msg::Header header) {
  header.frame_id = this->pub_odometry_frame_id;
  auto msg = RosNComWrapper::path(this->past_poses, header);
  pubPath_->publish(msg);
}

void OxtsIns::time_reference(std_msgs::msg::Header header) {
  header.frame_id = "oxts_link";
  auto msg = RosNComWrapper::time_reference(this->nrx, header);
  pubTimeReference_->publish(msg);
}

void OxtsIns::getLrf() {
  // Configured to come from NCom LRF, and the NCom LRF is valid.
  if (this->lrf_source == LRF_SOURCE::NCOM_LRF && nrx->mIsRefLatValid) {
    this->lrf = RosNComWrapper::getNcomLrf(nrx);
    this->lrf_valid = true;
  }
  // Configured to come from the first NCom packet
  else if (!this->lrf_valid && this->lrf_source == LRF_SOURCE::NCOM_FIRST) {
    this->lrf.origin(nrx->mLat, nrx->mLon, nrx->mAlt);
    // mHeading is in NED. Get angle between LRF and ENU
    this->lrf.heading((nrx->mHeading - 90) * NAV_CONST::DEG2RADS);
    this->lrf_valid = true;

  } else if (!this->lrf_valid &&
             this->lrf_source == LRF_SOURCE::NCOM_FIRST_ENU) {
    this->lrf.origin(nrx->mLat, nrx->mLon, nrx->mAlt);
    this->lrf.heading((0.0) * NAV_CONST::DEG2RADS); // LRF aligned to ENU
    this->lrf_valid = true;
 
  }
}

} // namespace oxts_ins
