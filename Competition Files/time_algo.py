target_time = 63
turn_num = 7
straight_num = 35.5
turn_time = 3.2 # time for one turn (in seconds)
straight_time = 0.93  # time for one straight at 50% speed (in seconds)
total_turn_time = turn_time * turn_num
remaining_time = target_time - total_turn_time
time_per_straight = remaining_time / straight_num
global average_speed
average_speed=0.5*(straight_time / time_per_straight)
print(average_speed)
print(time_per_straight)