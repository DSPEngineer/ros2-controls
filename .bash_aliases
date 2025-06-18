## Using these functions, you can use a target path in the command;
##   src dxmos/badl
## will move you to sub-directory dxmso/badl within your source tree.
#alias b='~/scripts/bcompare.sh'
##alias build='make -I $MAKE_RULES_PATH -f $MAKE_RULES_PATH/XKL_MakeCustom'
alias e='env | sort'
alias h='history'

alias p2='python2'
alias p3='python3'

# some 'ls' aliases
alias ls='ls --color=none'
alias lsl='ls -oal --color=none'
alias ll='ls -al --color=none'
alias la='ls -A --color=none'
alias l='ls -CF --color=none'
alias ls='ls --color=none'

#alias dir='dir --color=auto'
#alias vdir='vdir --color=auto'

alias grep='grep --color=auto'
alias fgrep='fgrep --color=auto'
alias egrep='egrep --color=auto'

###
function em()     { emacs $* & }
function root()   { cd $ENLISTMENT_ROOT/$1; }
function src()    { cd $SOURCE_ROOT/$1;  }
function bld()    { cd $BINARY_ROOT/$1;   }
function tools()  { cd $TOOLS_ROOT/$1;   }
function search() { find $SOURCE_ROOT/. ${2:+-iname "$2"} -exec grep -sH $1 {} \;;}


###########################################################
p ()
{
    #echo $PATH
    if ! [[ -v PATH ]]
    then
        echo "PATH not defined."
    else
        COUNT=`echo $PATH | tr -cd ':' | wc -c`
        echo Entries = $((COUNT+1))
        myPATH=`echo $PATH | sed 's/:/ /g' `
        #echo $myPATH
        for d in $myPATH ; do
            echo "-- $d"
        done
    fi
}


###########################################################
lib ()
{
    #echo $LD_LIBRARY_PATH
    if ! [[ -v LD_LIBRARY_PATH ]]
    then
        echo "LD_LIBRARY_PATH not defined."
    else
        #echo $LD_LIBRARY_PATH
        COUNT=`echo $LD_LIBRARY_PATH | tr -cd ':' | wc -c`
        echo Entries = $((COUNT+1))
        myPATH=`echo $LD_LIBRARY_PATH | sed 's/:/ /g' `
        #echo $myPATH
        for d in $myPATH
        do
            echo "-- $d"
        done
    fi

}
