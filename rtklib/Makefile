include $(shell rospack find mk)/cmake.mk

all: installed

BIN_DIR    = bin
INC_DIR    = include
LIB_DIR    = lib
GIT_DIR    = src
GIT_URL    = https://github.com/tomojitakasu/RTKLIB.git
include $(shell rospack find mk)/git_checkout.mk

built: $(GIT_DIR) patched
	cd $(GIT_DIR)/app && sh makeall.sh
	touch built

installed: built
	mkdir -p $(BIN_DIR)
	ln -f $(GIT_DIR)/app/pos2kml/gcc/pos2kml $(BIN_DIR)
	ln -f $(GIT_DIR)/app/str2str/gcc/str2str $(BIN_DIR)
	ln -f $(GIT_DIR)/app/rnx2rtkp/gcc/rnx2rtkp $(BIN_DIR)
	ln -f $(GIT_DIR)/app/convbin/gcc/convbin $(BIN_DIR)
	ln -f $(GIT_DIR)/app/rtkrcv/gcc/rtkrcv $(BIN_DIR)
	mkdir -p $(INC_DIR)/rtklib
	ln -f $(GIT_DIR)/src/rtklib.h $(INC_DIR)/rtklib
	touch installed

clean:
	$(RM) built
	$(RM) installed
	$(RM) -r $(GIT_DIR)

wipe: clean
	$(RM) -r $(BIN_DIR)
	$(RM) -r $(INC_DIR)
	$(RM) -r $(LIB_DIR)

