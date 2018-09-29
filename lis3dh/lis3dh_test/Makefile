#--------------------------------------------------------------
#               Quectel OpenLinux
#--------------------------------------------------------------
QL_SDK_PATH   ?= $(shell pwd)/../..

QL_EXP_TARGETS = lis3dh_test
QL_EXP_LDLIBS  =

#--------------------------------------------------------
# TOOLS BASIC CONFIG
# Note: No Need to change them
#--------------------------------------------------------
CPPFLAGS = -I./ -I$(QL_TOOLS_PATH)/mdm9607/usr/include -I$(QL_SDK_PATH)/include
CPPFLAGS += -Wall -Werror
LDFLAGS = -L./ -L$(QL_SDK_PATH)/lib -lrt ${QL_EXP_LDLIBS} -lpthread

TEST_TARGET_OBJ += lis3dh_test.o option.o

all: $(QL_EXP_TARGETS)
.PHPNY: all

%.o:%.c
	$(COMPILE.c) $^ $(CPPFLAGS)

$(QL_EXP_TARGETS):$(TEST_TARGET_OBJ)
	$(LINK.o) $(TEST_TARGET_OBJ) -o $@

clean:
	rm -rf $(QL_EXP_TARGETS) *.o

.PHONY:checkmake
checkmake:  
	@echo -e "CURDIR =		\n	${CURDIR}"  
	@echo -e "\nMAKE_VERSION =	\n	${MAKE_VERSION}"  
	@echo -e "\nMAKEFILE_LIST =	\n	${MAKEFILE_LIST}"  
	@echo -e "\nCOMPILE.c =		\n	${COMPILE.c}"
	@echo -e "\nCOMPILE.cc =	\n	${COMPILE.cc}"
	@echo -e "\nCOMPILE.cpp =	\n	${COMPILE.cpp}"
	@echo -e "\nLINK.cc =		\n	${LINK.cc}"
	@echo -e "\nLINK.o =		\n	${LINK.o}"
	@echo -e "\nCPPFLAGS =		\n	${CPPFLAGS}"
	@echo -e "\nCFLAGS =		\n	${CFLAGS}"
	@echo -e "\nCXXFLAGS =		\n	${CXXFLAGS}"
	@echo -e "\nLDFLAGS =		\n	${LDFLAGS}"
	@echo -e "\nLDLIBS =		\n	${LDLIBS}"