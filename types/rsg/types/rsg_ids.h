//#define MAX_NUMBER_OF_IDS 64 // Some arbitrary choice

struct rsg_ids {
	struct rsg_uuid ids[64]; //[MAX_NUMBER_OF_IDS]; //ffi can not parse this
	unsigned int numberOfIds;
};

