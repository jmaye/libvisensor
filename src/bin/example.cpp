/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the               *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

/** \file example.cpp
    \brief This file is a testing binary for the VI-sensor library.
  */

#include <unistd.h>

#include <iostream>
#include <vector>
#include <algorithm>

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include "visensor/visensor.hpp"

void frameCallback(visensor::ViFrame::Ptr frame_ptr,
    visensor::ViErrorCode error) {
}

void denseCallback(visensor::ViFrame::Ptr frame_ptr,
    visensor::ViErrorCode error) {
}

void imuCallback(boost::shared_ptr<visensor::ViImuMsg> imu_ptr,
    visensor::ViErrorCode error) {
}

void frameCornerCallback(visensor::ViFrame::Ptr frame_ptr,
    visensor::ViCorner::Ptr corners_ptr) {
}

void triggerCallback(visensor::ViExternalTriggerMsg::Ptr trigger_ptr) {
}

int main(int argc, char** argv) {
  visensor::ViSensorDriver driver;
  try {
    driver.init();
  }
  catch (const visensor::exceptions& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
  std::vector<visensor::SensorId::SensorId> sensorIds =
    driver.getListOfSensorIDs();
  std::vector<visensor::SensorId::SensorId> cameraIds =
    driver.getListOfCameraIDs();
  std::vector<visensor::SensorId::SensorId> denseIds =
    driver.getListOfDenseIDs();
  std::vector<visensor::SensorId::SensorId> imuIds = driver.getListOfImuIDs();
  std::vector<visensor::SensorId::SensorId> triggerIds =
    driver.getListOfTriggerIDs();
  try {
    driver.setCameraCallback(boost::bind(&frameCallback, _1, _2));
    driver.setDenseMatcherCallback(boost::bind(&denseCallback, _1, _2));
    driver.setImuCallback(boost::bind(&imuCallback, _1, _2));
    driver.setFramesCornersCallback(boost::bind(&frameCornerCallback, _1, _2));
    driver.setExternalTriggerCallback(boost::bind(&triggerCallback, _1));
    driver.setCameraCalibrationSlot(0);
  }
  catch (const visensor::exceptions& e) {
    std::cerr << e.what() << std::endl;
  }
  driver.startAllCameras();
  driver.startAllCorners();
  driver.startAllImus();
  driver.startAllExternalTriggers(IMU_FREQUENCY);
  driver.startAllDenseMatchers();
  if(std::count(sensorIds.begin(), sensorIds.end(),
      visensor::SensorId::LED_FLASHER0) > 0 )
    driver.startSensor(visensor::SensorId::LED_FLASHER0);
  sleep(10);
  return 0;
}
