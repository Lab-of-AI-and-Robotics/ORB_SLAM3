# ORB-SLAM3

### V1.0, December 22th, 2021
**Authors:** Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, [José M. M. Montiel](http://webdiis.unizar.es/~josemari/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/).

[original repo](https://github.com/UZ-SLAMLab/ORB_SLAM3.git)

# 1. License

ORB-SLAM3 is released under [GPLv3 license](https://github.com/UZ-SLAMLab/ORB_SLAM3/LICENSE). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Dependencies.md).

For a closed-source version of ORB-SLAM3 for commercial purposes, please contact the authors: orbslam (at) unizar (dot) es.

If you use ORB-SLAM3 in an academic work, please cite:
  
    @article{ORBSLAM3_TRO,
      title={{ORB-SLAM3}: An Accurate Open-Source Library for Visual, Visual-Inertial 
               and Multi-Map {SLAM}},
      author={Campos, Carlos AND Elvira, Richard AND G\´omez, Juan J. AND Montiel, 
              Jos\'e M. M. AND Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics}, 
      volume={37},
      number={6},
      pages={1874-1890},
      year={2021}
     }

# Modified part

## To publish all map points as a ros message
- include/System.h
```cpp
// line 175
    int GetTrackingState();
    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();
    // Add) riboha
    std::vector<MapPoint*> GetAllMapPoints(); // Added this line
```
- src/System.cc
```cpp
// line 1545
    return checksum;
}

vector<MapPoint*> System::GetAllMapPoints()
{
    Map* pActiveMap = mpAtlas->GetCurrentMap();
    return pActiveMap->GetAllMapPoints();
    // const vector<MapPoint*> &vpRefMPs = pActiveMap->GetReferenceMapPoints();
}
```

## Re-initializing
- src/LocalMapping.cc
```cpp
// line 133
if(dist>0.05)
mTinit += mpCurrentKeyFrame->mTimeStamp - mpCurrentKeyFrame->mPrevKF->mTimeStamp;

// commented out here
// if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA2())
// {
//     if((mTinit<10.f) && (dist<0.02))
//     {
//         cout << "Not enough motion for initializing. Reseting..." << endl;
//         unique_lock<mutex> lock(mMutexReset);
//         mbResetRequestedActiveMap = true;
//         mpMapToReset = mpCurrentKeyFrame->GetMap();
//         mbBadImu = true;
//     }
// }

bool bLarge = ((mpTracker->GetMatchesInliers()>75)&&mbMonocular)||((mpTracker->GetMatchesInliers()>100)&&!mbMonocular);
Optimizer::LocalInertialBA(mpCurrentKeyFrame, &mbAbortBA, mpCurrentKeyFrame->GetMap(),num_FixedKF_BA,num_OptKF_BA,num_MPs_BA,num_edges_BA, bLarge, !mpCurrentKeyFrame->GetMap()->GetIniertialBA2());
b_doneLBA = true;
```

## Re-localization in Localization mode

- src/Tracking.cc
```cpp
    // line 2039
    // if(mState==LOST)
    if(mState == LOST || mState == RECENTLY_LOST)
```