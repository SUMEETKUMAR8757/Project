# .bashrc

# Source global definitions
if [ -f /etc/bashrc ]; then
	. /etc/bashrc
alias g='gvim'
alias rm='rm -i'
alias cp='cp -i'
alias mv='mv -i'
alias c='clear'
export PATH="/usr/questasim/linux_x86_64/":$PATH
export PATH="/usr/questasim/RUVM_2021.2":$PATH
export LM_LICENSE_FILE="/usr/questasim/license.dat":$LM_LICENSE_FILE
export MGLS_LICENSE_FILE="/usr/questasim/mgcld.lic":$MGLS_LICENSE_FILE

fi

# Uncomment the following line if you don't like systemctl's auto-paging feature:
# export SYSTEMD_PAGER=

# User specific aliases and functions
alias g='gvim'
alias rm='rm -i'
alias cp='cp -i'
alias mv='mv -i'
alias c='clear'

