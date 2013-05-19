ifndef DEVELOPMENT_DIR
  DEVELOPMENT_DIR := $(realpath .)
endif

ifndef OBJ_DIR
  OBJ_DIR := $(DEVELOPMENT_DIR)/objects
endif



SRC_DIR := $(DEVELOPMENT_DIR)/src
TEST_DIR := $(DEVELOPMENT_DIR)/test
MKDIR := mkdir
RANLIB := ranlib
LD := $(CXX)

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

CXXFLAGS += -ggdb3 -Wall --std=c++11

LIB_SRCS := $(wildcard $(SRC_DIR)/*.cc)
LIB_INCLUDE_DIRS := -I$(SRC_DIR)
LIB_OBJS := $(call obj-dir-transform,$(LIB_SRCS))
LIB_CXXFLAGS := $(CXXFLAGS) $(LIB_INCLUDE_DIRS)
LIB_NAME := zengarden.a

$(LIB_OBJS): override CXXFLAGS = $(LIB_CXXFLAGS)

-include $(LIB_OBJS:.o=.P)

$(LIB_NAME): $(SRC_DIR) $(LIB_OBJS)
	$(AR) -ruvs $@ $(filter-out $<,$?)
	$(RANLIB) $@

TEST_SRCS := $(wildcard $(TEST_DIR)/*.cc) 
TEST_INCLUDE_DIRS := -I$(SRC_DIR) -I$(TEST_DIR) -I$(QUICKCHECK_DIR)
TEST_OBJS = $(call obj-dir-transform,$(TEST_SRCS))
TEST_CXXFLAGS := $(CXXFLAGS) $(TEST_INCLUDE_DIRS)
TEST_BINARY := zentest

$(TEST_OBJS): override CXXFLAGS = $(TEST_CXXFLAGS)

-include $(LIB_OBJS:.o=.P)

$(TEST_BINARY): $(TEST_OBJS) $(LIB_NAME)
	$(LD) $(LDFLAGS) $^ -o $(TEST_BINARY)


clean:
	rm -rf $(OBJ_DIR)
	rm -f $(LIB_NAME)

all: $(LIB_NAME)

test: $(TEST_BINARY)

run-test: $(TEST_BINARY)
	./$(TEST_BINARY)
