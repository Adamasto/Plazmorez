#ifndef SWITCHER_CONTROL_H
#define SWITCHER_CONTROL_H

struct byteSwitcherH
{
	unsigned X_backward:1;
	unsigned X_forward:1;
	unsigned Y_left_backward:1;
	unsigned Y_left_forward:1;
	unsigned Y_right_backward:1;
	unsigned Y_right_forward:1;
	unsigned reserved1:1;
	unsigned reserved2:1;
};




#endif /* SWITCHER_CONTROL_H */