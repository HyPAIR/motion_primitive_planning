# TODO

## Lift
lift in mur launch, but not mir
- load lift_module in mur.lauch
- connect mir->lift->UR or mir->ur in mur.launch

## HW/Sim identic
same urdf for hw and sim
    - remap robot_description
    - robot_state_publisher
        - change tf_prefix to ""
    - joint_state_publisher
        - change tf_prefix to "UR16_l/"