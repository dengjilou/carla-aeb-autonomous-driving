set pagination off
set debuginfod enabled off
set breakpoint pending on
start
break control_step if input != 0 && input->dt <= 0
continue
print input->dt
bt
finish
print $eax
continue
