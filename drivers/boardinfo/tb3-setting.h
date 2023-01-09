#ifndef TB3_SETTING_H
#define TB3_SETTING_H

int tb3_gpios(struct device *);

void tb3_gpios_free(void);

int tb3_adcs(struct device *, const char *, int *, int *, int *);

#endif
