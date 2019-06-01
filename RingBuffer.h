/***** RingBuffer.h *****/
//
// © copyright Toby Gifford 2011
//
//

#ifndef __RING_BUFFER__
#define __RING_BUFFER__

inline int ybot_mod(int x, int b) { return x % b; }

template <class T>
class ShiftingPool {
	
public:
	ShiftingPool();                                                     /**< Constructor */
	ShiftingPool(int N);                                                /**< Constructor */
	ShiftingPool(int N, const T& x);                                    /**< Constructor */
	ShiftingPool(const ShiftingPool<T>& p);                    			/**< Copy Constructor */         
	ShiftingPool& operator=(const ShiftingPool<T>&);       				/**< Assignment Constructor */ 
	~ShiftingPool();                                                    /**< Destructor */ 
	
private:
	T& at(int j) const { return pool_[j]; }								/**< IMPORTANT - This is meant to be private! Use [] operator for element access. */
public:																	// this function is here to allow internal access to the actual array, but does not respect the shifting		
	T& operator[](int j) { return pool_[ybot_mod(S_ + j, N_)]; }   		/**< Element access */
	T& next();                                                          /**< Iterator */
	T& last() { return pool_[ybot_mod(S_+ L_-1, N_)]; }                 /**< Access last element */
	//T& accessLast() { return pool_[ybot_mod(S_ + L_-1, N_)]; }
	T& first() { return pool_[S_]; }                                    /**< Access first element */     
	
	void reset() { L_ = 0; S_ = 0; }                                    /**< Clear out data */
	void add(const T& x);                                               /**< Shift in new element at end */
	void shift(T& in, T* out);											/**< Shift in new element at end and shift out the first element*/	
	int length() const {return L_; }                                    /**< The current length */      
	int capacity() const {return N_; }                                  /**< The maximum length */
	void changeCapacity(int N, bool copy=true);                 		/**< Change the maximum length */
	
	void push() { (L_ < N_-1) ? ++L_ :  S_ = ybot_mod(S_+1, N_); }   	/**< Extends the length by one */
	void push(T& x) { add(x);}                                          /**< Same as add() */
	void pop() { if(L_ >  0) --L_; }                                    /**< Decreases the length by one (i.e. deletes last element) */
	void cull() { if(L_ > 0) --L_, S_ = ybot_mod(S_+1, N_);}            /**< Deletes first element */ 
	
private:
	T* pool_;	
	int N_;					/**< N is the number of objects in the pool */
	int L_;                     /**< L is the current length of valid pool members */
	int S_;					/**< S is the current start index of the first valid pool member */
};


/************ Implementation *************/
template <class T>
ShiftingPool<T>::ShiftingPool() : N_(0), L_(0), S_(0) {
	pool_ = new T[0];
}

template <class T>
ShiftingPool<T>::ShiftingPool(int N) : N_(N), L_(0), S_(0) {
	pool_ = new T[N];
}

template <class T>
ShiftingPool<T>::ShiftingPool(const ShiftingPool<T>& p) {
	pool_ = new T[p.capacity()];
	N_ = p.N_;
	L_ = p.L_;
	S_ = 0;
	for (int j=0; j < L_; j++)
		pool_[j] = p.pool_[ybot_mod(p.S_ + j, p.N_)];
	//pool_[j] = p.at(j);
}


template <class T>
ShiftingPool<T>::ShiftingPool(int N, const T& x) : N_(N), L_(N), S_(0) {
	pool_ = new T[N];
	for (int j=0; j < N_; j++)
		pool_[j] = x;
}

template <class T>
ShiftingPool<T>& ShiftingPool<T>::operator=(const ShiftingPool<T>& p) {
	if (this != &p) {
		N_ = p.N_;
		L_ = p.L_;
		S_ = 0;
		T* new_pool_= new T[N_];
		for (int j=0; j<L_; j++)
			new_pool_[j] = p.pool_[ybot_mod(p.S_ + j, p.N_)];
		//new_pool_[j] = p.at(j);
		
		delete[] pool_;
		pool_ = new_pool_;
		
	}
	return *this;
}


template <class T>
void ShiftingPool<T>::changeCapacity(int N, bool copy) {
	T* new_pool_= new T[N];
	if (L_ < N) {
		if (copy)
			for (int j=0; j<L_; j++)
				new_pool_[j] = pool_[ybot_mod(S_ + j, N_)];
		else
			L_ = 0;
		delete[] pool_;
		pool_ = new_pool_;
		N_ = N;
		S_ = 0;
	} else {
		if (copy) {
			for (int j=0; j<N; j++)
				new_pool_[N-j-1] = pool_[ybot_mod(S_ + L_- j -1, N_)];
			L_ = N;
		} else { 
			L_ = 0;
		}	
		N_ = N;
		S_ = 0;
		
	}	
}


template <class T>
ShiftingPool<T>::~ShiftingPool() {
	if (pool_ != NULL) {
		delete[] pool_;
	}
}

template <class T>
void ShiftingPool<T>::add(const T& x) {
	if (N_ == 0)
		return;
	
	if (L_ < N_) {
        
		// There is a spare slot remaining for this value
		int E_ = (S_ + L_++) % N_;
		pool_[E_] = x;
        
	} else if (L_ == N_) {
		
		// No slots left, wrap around	
		S_ = (S_+1) % N_; 
		int E_ = (S_ + L_ - 1) % N_;
		pool_[E_] = x;	
        
	}
	
}


template <class T>
void ShiftingPool<T>::shift(T& in, T* out) {
	*out = first();
	add(in);
}


template <class T>
T& ShiftingPool<T>::next() {
	if (L_ < N_) {
		return pool_[ybot_mod(L_++ + S_, N_)];
	} else {
		S_ = ybot_mod(S_+1, N_);
		return pool_[ybot_mod(L_ + S_- 1, N_)];
	}
}

#endif