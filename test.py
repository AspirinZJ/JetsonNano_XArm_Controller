import moveServo


def list2str(lst):
    str_lst = [str(x) for x in lst]
    string = ' '.join(str_lst)
    return string


time = 1000
position = [500] * 6
position_str = list2str(position)
moveServo.moveServo(position_str, 1000)