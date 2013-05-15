ifndef DEVELOPMENT_DIR
  DEVELOPMENT_DIR := $(realpath .)
endif

ifndef OBJ_DIR
  OBJ_DIR := $(DEVELOPMENT_DIR)/objects
endif

SRC_DIR := $(DEVELOPMENT_DIR)/src
MKDIR := mkdir
RANLIB := ranlib

$(OBJ_DIR):
	$(MKDIR) -p $(OBJ_DIR)

$(OBJ_DIR)/%.o : %.cc
	@echo $(CXX) $< -o $@
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -MD -c $< -o $@
	@cp $(@D)/$(@F:.o=.d) $(@D)/$(@F:.o=.P); \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
	    -e '/^$$/ d' -e 's/$$/ :/' < $(@D)/$(@F:.o=.d) >> $(@D)/$(@F:.o=.P); \
	rm -f $(@D)/$(@F:.o=.d)

obj-dir-transform = $(foreach src,$(1), $(subst $(DEVELOPMENT_DIR),$(OBJ_DIR), $(src:%.cc=%$(EXTENSION).o)))


EXTERN_DIR := $(DEVELOPMENT_DIR)/extern
QUICKCHECK_DIR := $(EXTERN_DIR)/quickcheck

CXXFLAGS += -ggdb3 -Wall --std=c++0x

LIB_SRCS := $(wildcard $(SRC_DIR)/*.cc)
LIB_INCLUDE_DIRS := $(SRC_DIR)
LIB_OBJS := $(call obj-dir-transform,$(LIB_SRCS))
LIB_CXXFLAGS := $(CXXFLAGS) $(LIB_INCLUDE_DIRS)
LIB_NAME := zengarden.a

$(LIB_OBJS): override CXXFLAGS = $(LIB_CXXFLAGS)

$(LIB_NAME): $(SRC_DIR) $(LIB_OBJS)
	$(AR) -ruvs $@ $(filter-out $<,$?)
	$(RANLIB) $@

clean:
	rm -rf $(OBJ_DIR)
	rm -f $(LIB_NAME)
