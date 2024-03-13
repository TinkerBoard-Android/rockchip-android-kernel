#ifndef TB3N_SETTING_H
#define TB3N_SETTING_H

int tb3n_gpios(struct device *);

void tb3n_gpios_free(void);

int tb3n_adcs(struct device *, const char *, int *, int *, int *);

#endif
