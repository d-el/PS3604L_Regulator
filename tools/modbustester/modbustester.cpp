#include <iostream>
using namespace std;
#include <modbus.h>
#include <plog.h>
#include <hexdump.h>
#include <unistd.h>
#include <prmSystem.h>
#include <string.h>

#define LOG_LOCAL_LEVEL P_LOG_VERBOSE /*P_LOG_ERROR P_LOG_WARN P_LOG_VERBOSE*/
static const char *logTag = "modbustester";

typedef enum { FILE_READ, FILE_WRITE, REG_WRITE, REG_NUM_READ, REG_READ } programMode_t;

/*!****************************************************************************
 * @brief
 */
static const prmHandle_type *getHandlerByAddress(uint16_t address){
	size_t i = 0;
	while(1){
		const prmHandle_type *ph = prm_getHandler(i++);
		if(ph == NULL){
			break;
		}
		if(ph->addr == address){
			return ph;
		}
	}

	return NULL;
}

int main(int argc, char *argv[]) {
	setvbuf(stdout, NULL, _IONBF, 0);	//not buffered

	if(argc < 5){
		P_LOGE(logTag, "argument error");
		return 1;
	}
	char *device = argv[1];
	uint32_t baud;
	if(!sscanf(argv[2], "%u", &baud)){
		P_LOGE(logTag, "error baud");
		return 1;
	}
	uint32_t slave;
	if(!sscanf(argv[3], "%u", &slave)){
		P_LOGE(logTag, "error slave");
		return 1;
	}

	programMode_t mode = REG_READ;
	int address = 0;
	char *regval = NULL;
	if(strcmp("rread", argv[4]) == 0){
		mode = REG_READ;
	}
	else if(strcmp("rreadn", argv[4]) == 0){
		if(argc != 6){
			P_LOGE(logTag, "error argument");
			return 1;
		}
		if(!sscanf(argv[5], "%x", &address)){
			P_LOGE(logTag, "error reg address");
			return 1;
		}
		mode = REG_NUM_READ;
	}
	else if(strcmp("fread", argv[4]) == 0){
		mode = FILE_READ;
	}
	else if(strcmp("fwrite", argv[4]) == 0){
		mode = FILE_WRITE;
	}
	else if(strcmp("rwrite", argv[4]) == 0){
		if(!sscanf(argv[5], "%x", &address)){
			P_LOGE(logTag, "error reg address");
			return 1;
		}
		regval = argv[6];
		mode = REG_WRITE;
	}else{
		P_LOGE(logTag, "error mode");
		return 1;
	}

	int fileNumber = 0;
	const char* filename = NULL;
	if(mode == FILE_READ || mode == FILE_WRITE){
		if(argc != 7){
			P_LOGE(logTag, "error argument");
			return 1;
		}
		if(!sscanf(argv[5], "%u", &fileNumber)){
			P_LOGE(logTag, "error fileNumber");
			return 1;
		}
		filename = argv[6];
	}

	time_t rawtime = time(NULL);

	const uint16_t closeFileAddr = 0xFF00;
	const uint32_t recordLen = 64;

	modbus_t *ctx;
	ctx = modbus_new_rtu(device, baud, 'N', 8, 1);
	if(ctx == NULL){
		P_LOGW(logTag, "error create modbus");
	}
	modbus_set_slave(ctx, slave);
	modbus_set_debug(ctx, FALSE);
	modbus_connect(ctx);

	if(mode == FILE_READ){
		P_LOGI(logTag, "file read");

		FILE *fout = fopen(filename, "w");
		if(fout == NULL){
			modbus_close(ctx);
			P_LOGW(logTag, "error create file %s", filename);
			return 1;
		}

		size_t fileSize = 0;
		int status = modbus_read_registers(ctx, 0xFE00 + fileNumber * 2, 2, (uint16_t*)&fileSize);
		if(status > 0){
			P_LOGI(logTag, "file size: %zu B",  fileSize);
		}
		else{
			P_LOGW(logTag, "error: %s",  modbus_strerror(errno));
		}

		uint32_t recordNumber = (fileSize + 1) / 2;
		for(size_t record = 0; record < recordNumber; record += recordLen){
			uint8_t buffer[256];
			uint8_t currentrecordnum = (recordNumber - record) > recordLen ? recordLen : recordNumber - record;
			status = modbus_read_file_record(ctx, fileNumber, record / recordLen, currentrecordnum, (uint16_t*)&buffer[0]);
			if(status > 0){
				fwrite(buffer, 1, currentrecordnum * 2, fout);
			}else{
				P_LOGW(logTag, "error: %s",  modbus_strerror(errno));
				break;
			}

			time_t times = time(NULL) - rawtime;
			size_t bytes = record * 2;
			size_t speed = times == 0 ? 0 : bytes / times;
			printf("\r%zu/%zu %us %zuB/s", bytes, fileSize, (uint32_t)times, speed);
		}
		printf("\n");

		status = modbus_write_register(ctx, closeFileAddr + fileNumber, 1);
		if(status != 1){
			P_LOGW(logTag, "Close file error: %s ", modbus_strerror(errno));
		}

		fclose(fout);
	}
	else if(mode == FILE_WRITE){
		P_LOGI(logTag, "file write");
		FILE *fin = fopen(filename, "r");
		if(fin == NULL){
			modbus_close(ctx);
			P_LOGW(logTag, "error open file %s", filename);
			return 1;
		}

		fseek(fin, 0, SEEK_END);
		size_t fileSize = ftell(fin);
		fseek(fin, 0, SEEK_SET);
		P_LOGI(logTag, "file size: %zu B",  fileSize);

		uint32_t recordNumber = (fileSize + 1) / 2;
		for(size_t record = 0; record < recordNumber; record += recordLen){
			uint8_t buffer[256];
			memset(buffer, 0, sizeof(buffer));
			uint8_t currentrecordnum = (recordNumber - record) > recordLen ? recordLen : recordNumber - record;
			size_t readded = fread(buffer, 1, currentrecordnum * 2, fin);
			int status = modbus_write_file_record(ctx, fileNumber, record / recordLen, currentrecordnum, (uint16_t*)&buffer[0]);
			if(status > 0){

			}else{
				P_LOGW(logTag, "error: %s",  modbus_strerror(errno));
				break;
			}

			time_t times = time(NULL) - rawtime;
			size_t bytes = record * 2;
			size_t speed = times == 0 ? 0 : bytes / times;
			printf("\r%zu/%zu %us %zuB/s ", bytes, fileSize, (uint32_t)times, speed);
		}
		printf("\n");

		int status = modbus_write_register(ctx, closeFileAddr, 0);
		if(status != 1){
			P_LOGW(logTag, "Close file error: %s", modbus_strerror(errno));
		}

		fclose(fin);
	}
	else if(mode == REG_WRITE){
		P_LOGI(logTag, "register write");
		const prmHandle_type *ph = getHandlerByAddress(address);
		if(ph == NULL){
			P_LOGW(logTag, "error register address");
			modbus_close(ctx);
			return 1;
		}

		prm_toPrm(regval, ph);

		uint8_t regs;
		switch(prm_getSize(ph)){
			case 1:
			case 2:
				regs = 1;
				break;
			case 4:
				regs = 2;
		}
		int status = modbus_write_registers(ctx, ph->addr, regs, (uint16_t*)&ph->prm->t_s16Frmt);

		if(status == regs){
			char string[128];
			prm_toString(string, sizeof(string), ph);
			P_LOGI(logTag, "[%04Xh] Write %s: %s %s", ph->addr, ph->label, string, ph->units);
		}
		else{
			P_LOGW(logTag, "[%04Xh] %s: Write failed, error: %s", ph->addr, ph->label, modbus_strerror(errno));
		}
	}
	else if(mode == REG_NUM_READ){
		P_LOGI(logTag, "register number read");
		const prmHandle_type *ph = getHandlerByAddress(address);
		if(ph == NULL){
			P_LOGW(logTag, "error register address");
			modbus_close(ctx);
			return 1;
		}
		uint8_t regs;
		switch(prm_getSize(ph)){
			case 1:
			case 2:
				regs = 1;
				break;
			case 4:
				regs = 2;
		}

		int status = modbus_read_registers(ctx, ph->addr, regs, (uint16_t*)&ph->prm->t_s16Frmt);
		if(status == regs){
			char string[128];
			prm_toString(string, sizeof(string), ph);
			P_LOGI(logTag, "[%04Xh] Read %s: %s %s", ph->addr, ph->label, string, ph->units);
		}
		else{
			P_LOGW(logTag, "[%04Xh] %s: Read failed, error: %s", ph->addr, ph->label, modbus_strerror(errno));
		}
	}
	else{
		P_LOGI(logTag, "register read");
		for(size_t i = 0; i < endOfNumberPrm; i++){
			const prmHandle_type *ph = prm_getHandler(i);
			uint8_t regs;
			switch(prm_getSize(ph)){
				case 1:
				case 2:
					regs = 1;
					break;
				case 4:
					regs = 2;
			}

			int status = modbus_read_registers(ctx, ph->addr, regs, (uint16_t*)&ph->prm->t_s16Frmt);
			if(status == regs){
				char string[128];
				prm_toString(string, sizeof(string), ph);
				P_LOGI(logTag, "[%04Xh] Read %s: %s %s", ph->addr, ph->label, string, ph->units);
			}
			else{
				P_LOGW(logTag, "[%04Xh] %s: Read failed, error: %s", ph->addr, ph->label, modbus_strerror(errno));
			}
		}
	}

	modbus_close(ctx);
	P_LOGI(logTag, "done");
	return 0;
}

extern "C"{
void setmode(const struct prmHandle* p, void *arg){}
void vsave(const struct prmHandle* p, void *arg){}
void isave(const struct prmHandle* p, void *arg){}
}

