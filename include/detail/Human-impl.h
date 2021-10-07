//
// Created by root on 10/7/21.
//

#ifndef LIBHUMAN_HUMAN_IMPL_H
#define LIBHUMAN_HUMAN_IMPL_H

namespace human {

    template<typename PostProcessor>
    void Human::setDefaultPostProcessor(
            const Eigen::VectorXd &velocityLimits,
            const Eigen::VectorXd &accelerationLimits,
            const typename PostProcessor::Params &params) {

        // See if the passed-in values aree valid or not
        bool velLimitsInvalid
                = (velocityLimits.squaredNorm() == 0.0)
                  || ((std::size_t) velocityLimits.size() != mMetaSkeleton->getNumDofs()
                      && (std::size_t) velocityLimits.size()
                         != mRightArm->getMetaSkeleton()->getNumDofs());
        auto vLimit = velLimitsInvalid ? mDefaultVelocityLimits : velocityLimits;
        mSoftVelocityLimits.segment(0, vLimit.size()) = vLimit;

        bool accLimitsInvalid = (accelerationLimits.squaredNorm() == 0.0)
                                || ((std::size_t) accelerationLimits.size()
                                    != mMetaSkeleton->getNumDofs()
                                    && (std::size_t) accelerationLimits.size()
                                       != mRightArm->getMetaSkeleton()->getNumDofs());
        auto aLimit
                = accLimitsInvalid ? mDefaultAccelerationLimits : accelerationLimits;
        mSoftAccelerationLimits.segment(0, aLimit.size()) = aLimit;

        // Update whole-arm postprocessor
        auto postprocessor = std::make_shared<PostProcessor>(
                mSoftVelocityLimits, mSoftAccelerationLimits, params);
        Robot::setDefaultPostProcessor(postprocessor);

        // Update arm-only post-processor
        postprocessor = std::make_shared<PostProcessor>(
                mSoftVelocityLimits.segment(0, mRightArm->getMetaSkeleton()->getNumDofs()),
                mSoftAccelerationLimits.segment(0, mRightArm->getMetaSkeleton()->getNumDofs()),
                params);
        mRightArm->setDefaultPostProcessor(postprocessor);

        // Update hand-only post-processor
        postprocessor = std::make_shared<PostProcessor>(
                mSoftVelocityLimits.segment(
                        mRightArm->getMetaSkeleton()->getNumDofs(),
                        mHandRobot->getMetaSkeleton()->getNumDofs()),
                mSoftAccelerationLimits.segment(
                        mRightArm->getMetaSkeleton()->getNumDofs(),
                        mHandRobot->getMetaSkeleton()->getNumDofs()),
                params);
        mHandRobot->setDefaultPostProcessor(postprocessor);
    }

}

#endif //LIBHUMAN_HUMAN_IMPL_H
