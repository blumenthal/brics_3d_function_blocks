#define MAX_NUMBER_OF_IDS 64 // Some arbitrary choice

struct rsg_ids {
	struct rsg_uuid ids[MAX_NUMBER_OF_IDS];
	unsigned int numberOfIds;
};
