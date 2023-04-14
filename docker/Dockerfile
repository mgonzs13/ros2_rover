FROM ubuntu:22.04

USER root
ARG DEBIAN_FRONTEND=noninteractive

COPY ./src/install/utils.sh $INST_SCRIPTS/
RUN chmod +x $INST_SCRIPTS/utils.sh && $INST_SCRIPTS/utils.sh

COPY ./src/install/ros2.sh $INST_SCRIPTS/
RUN chmod +x $INST_SCRIPTS/ros2.sh && $INST_SCRIPTS/ros2.sh

COPY ./src/install/ros2_utils.sh $INST_SCRIPTS/
RUN chmod +x $INST_SCRIPTS/ros2_utils.sh && $INST_SCRIPTS/ros2_utils.sh

COPY ./src/install/ros2_rover.sh $INST_SCRIPTS/
RUN chmod +x $INST_SCRIPTS/ros2_rover.sh && $INST_SCRIPTS/ros2_rover.sh
