all: installed

BASE_URL   = http://www.rtklib.com/prog
BUILD_DIR  = build
SOURCE_DIR = src
BIN_DIR    = bin

########
## Install from tarball
TARBALL     = $(BUILD_DIR)/rtklib_2.4.1.zip
TARBALL_URL = $(BASE_URL)/rtklib_2.4.1.zip
INITIAL_DIR = build/rtklib_2.4.1
UNPACK_CMD  = unzip
MD5SUM_FILE = rtklib_2.4.1.zip.md5sum
include $(shell rospack find mk)/download_unpack_build.mk
########

PATCHES = p1 p2 p3 p4 p5 p6 p7

# Download patch
$(BUILD_DIR)/rtklib_2.4.1_%.zip:
	`rospack find rosbuild`/bin/download_checkmd5.py $(BASE_URL)/$(@F) $@

# Apply patch
$(SOURCE_DIR)/rtklib_2.4.1_%.txt: $(BUILD_DIR)/rtklib_2.4.1_%.zip
	cd $(BUILD_DIR) && $(UNPACK_CMD) rtklib_2.4.1_$*.zip
	cp -r $(BUILD_DIR)/rtklib_2.4.1_$*/* $(SOURCE_DIR)
	$(RM) -rf $(BUILD_DIR)/rtklib_2.4.1_$*

patch: $(patsubst %,$(SOURCE_DIR)/rtklib_2.4.1_%.txt,$(PATCHES))

built: $(SOURCE_DIR)/unpacked patch
	cd $(SOURCE_DIR)/app && sh makeall.sh
	touch built

installed: built
	mkdir -p $(BIN_DIR)
	ln -f $(SOURCE_DIR)/app/pos2kml/gcc/pos2kml $(BIN_DIR)
	ln -f $(SOURCE_DIR)/app/str2str/gcc/str2str $(BIN_DIR)
	ln -f $(SOURCE_DIR)/app/rnx2rtkp/gcc/rnx2rtkp $(BIN_DIR)
	ln -f $(SOURCE_DIR)/app/convbin/gcc/convbin $(BIN_DIR)
	ln -f $(SOURCE_DIR)/app/rtkrcv/gcc/rtkrcv $(BIN_DIR)
	touch installed

clean:
	$(RM) built
	$(RM) installed
	$(RM) -r $(SOURCE_DIR)

wipe: clean
	$(RM) -r $(BUILD_DIR)
	$(RM) -r $(BIN_DIR)

