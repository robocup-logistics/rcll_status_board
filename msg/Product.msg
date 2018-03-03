# order id
int8 id

# complexity 0=C0 1=C1 2=C2 3=C3
int8 complexity

# base 1=BASE_RED 2=BASE_BLACK 3=BASE_SILVER
int8 base

# rings 0=NO_RING 1=RING_BLUE 2=RING_GREEN 3=RING_ORANGE 4=RING_YELLOW
int8 ring1
int8 ring2
int8 ring3

# cap 1=CAP_BLACK 2=CAP_GREY
int8 cap

# base 0=not started 1=construction 2=delivery 3=completed
# index: 0=base 1=ring1 2=ring2 3=ring3 4=cap 5=product
int8[] step_status

# product percentage
float32 percentage

# end delivery time
int8 end_delivery_time

# points
int8 points
int8 points_max
