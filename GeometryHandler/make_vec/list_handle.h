void downheap(int n, heap_data a[], int k); //n‘” a”z—ñ ’–Ú”Ô†
void upheap(int n, heap_data a[], int k); //n‘” a”z—ñ ’–Ú”Ô†

void add_element_to_list(list_data *tl,void *t);
void add_any_element_to_list(list_data *tl,void *t);
void del_element_from_list(list_data *tl,void *t);
void delete_list(list_data *tl); //list‚ğíœ‚·‚éB
void *load_list_elements(list_data *tl,int cleartrue);
void *load_list_element_and_next_pointer(list_data **tl);


#ifdef MAKE_CLUSTER_HIERARCHY
ring_list_data* add_element_to_ring_list(ring_list_data *rl,void *e, int type, object_data *o);
#else
ring_list_data* add_element_to_ring_list(ring_list_data *rl,void *e, int type);
#endif
ring_list_data* del_element_from_ring_list(ring_list_data *rl,void *e);
ring_list_data* del_list_from_ring_list(ring_list_data *rl,ring_list_data *r);
void print_ring_list(ring_list_data *l);
int return_number_of_ring_list(ring_list_data *l);

void make_unit_2object(object_data *bo,object_data *wo,int number_of_object);

void upheap(int n, heap_data a[], int k);
void downheap(int n, heap_data a[], int k);
