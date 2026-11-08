"""Resolve explicit bus IDs without connecting to hardware."""


def resolve_motor_ids(joints, overrides=None):
    overrides = overrides or {}
    unknown = set(overrides) - set(joints)
    if unknown:
        raise ValueError(f'Motor IDs supplied for unknown joints: {sorted(unknown)}')
    ids = {joint: overrides.get(joint, index + 1) for index, joint in enumerate(joints)}
    if any(type(value) is not int or not 0 <= value <= 252 for value in ids.values()):
        raise ValueError('Dynamixel motor IDs must be integers from 0 to 252')
    if len(set(ids.values())) != len(ids):
        raise ValueError('Motor IDs must be unique, including joints using default IDs')
    return ids
