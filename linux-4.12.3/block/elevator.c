/*
 *  Block device elevator/IO-scheduler.
 *
 *  Copyright (C) 2000 Andrea Arcangeli <andrea@suse.de> SuSE
 *
 * 30042000 Jens Axboe <axboe@kernel.dk> :
 *
 * Split the elevator a bit so that it is possible to choose a different
 * one or even write a new "plug in". There are three pieces:
 * - elevator_fn, inserts a new request in the queue list
 * - elevator_merge_fn, decides whether a new buffer can be merged with
 *   an existing request
 * - elevator_dequeue_fn, called when a request is taken off the active list
 *
 * 20082000 Dave Jones <davej@suse.de> :
 * Removed tests for max-bomb-segments, which was breaking elvtune
 *  when run without -bN
 *
 * Jens:
 * - Rework again to work with bio instead of buffer_heads
 * - loose bi_dev comparisons, partition handling is right now
 * - completely modularize elevator setup and teardown
 *
 */
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/blkdev.h>
#include <linux/elevator.h>
#include <linux/bio.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/compiler.h>
#include <linux/blktrace_api.h>
#include <linux/hash.h>
#include <linux/uaccess.h>
#include <linux/pm_runtime.h>
#include <linux/blk-cgroup.h>

#include <trace/events/block.h>

#include "blk.h"
#include "blk-mq-sched.h"
#include "blk-wbt.h"

static DEFINE_SPINLOCK(elv_list_lock);
static LIST_HEAD(elv_list);//yyf: 注册elevator_type的全局链表

/*
 * Merge hash stuff.
 */
#define rq_hash_key(rq)		(blk_rq_pos(rq) + blk_rq_sectors(rq)) //yyf: 算出是结束扇区号

/*
 * Query io scheduler to see if the current process issuing bio may be
 * merged with rq.
 */
static int elv_iosched_allow_bio_merge(struct request *rq, struct bio *bio)
{
	struct request_queue *q = rq->q;
	struct elevator_queue *e = q->elevator;
    
//yyf: 主要是回调elevator的钩子接口来判断是否允许merge
	if (e->uses_mq && e->type->ops.mq.allow_merge)
		return e->type->ops.mq.allow_merge(q, rq, bio);
	else if (!e->uses_mq && e->type->ops.sq.elevator_allow_bio_merge_fn)
		return e->type->ops.sq.elevator_allow_bio_merge_fn(q, rq, bio);

	return 1;
}

/*
 * can we safely merge with this request?
 */
bool elv_bio_merge_ok(struct request *rq, struct bio *bio)
{
	if (!blk_rq_merge_ok(rq, bio))//yyf: 先看块层是否允许合并
		return false;

	if (!elv_iosched_allow_bio_merge(rq, bio))//yyf: 再看elevator是否允许合并
		return false;

	return true;
}
EXPORT_SYMBOL(elv_bio_merge_ok);

static struct elevator_type *elevator_find(const char *name)
{
	struct elevator_type *e;

	list_for_each_entry(e, &elv_list, list) {//yyf: 遍历elv_list链表，查找相同名字的elevator_type
		if (!strcmp(e->elevator_name, name))
			return e;
	}

	return NULL;
}

static void elevator_put(struct elevator_type *e)
{
	module_put(e->elevator_owner);//yyf: 释放模块引用计数
}

static struct elevator_type *elevator_get(const char *name, bool try_loading)
{
	struct elevator_type *e;

	spin_lock(&elv_list_lock);

	e = elevator_find(name);
	if (!e && try_loading) {//yyf: 如果没有找到elevator，且try_loading设置时，尝试加载对应模块
		spin_unlock(&elv_list_lock);
		request_module("%s-iosched", name);
		spin_lock(&elv_list_lock);
		e = elevator_find(name);
	}

	if (e && !try_module_get(e->elevator_owner))//yyf: 找到elevator_type后，get下模块索引
		e = NULL;

	spin_unlock(&elv_list_lock);

	return e;
}

static char chosen_elevator[ELV_NAME_MAX];//yyf: 保存选择的elevator算法

static int __init elevator_setup(char *str)
{
	/*
	 * Be backwards-compatible with previous kernels, so users
	 * won't get the wrong elevator.
	 */
	strncpy(chosen_elevator, str, sizeof(chosen_elevator) - 1);
	return 1;
}

__setup("elevator=", elevator_setup);//yyf: 启动参数可以设置elevator是哪个

/* called during boot to load the elevator chosen by the elevator param */
void __init load_default_elevator_module(void)
{
	struct elevator_type *e;

	if (!chosen_elevator[0])
		return;

	spin_lock(&elv_list_lock);
	e = elevator_find(chosen_elevator);//yyf: 默认是从chosen_elevator名字查找elevator_type
	spin_unlock(&elv_list_lock);

	if (!e)
		request_module("%s-iosched", chosen_elevator);
}

static struct kobj_type elv_ktype;

struct elevator_queue *elevator_alloc(struct request_queue *q,
				  struct elevator_type *e)
{
	struct elevator_queue *eq;

	eq = kzalloc_node(sizeof(*eq), GFP_KERNEL, q->node);//yyf: slab申请elevator_queue
	if (unlikely(!eq))
		return NULL;
//yyf: 初始化elevator_queue，该结构是连接request_queue和elevator_type的
	eq->type = e;
	kobject_init(&eq->kobj, &elv_ktype);
	mutex_init(&eq->sysfs_lock);
	hash_init(eq->hash);
	eq->uses_mq = e->uses_mq;

	return eq;
}
EXPORT_SYMBOL(elevator_alloc);

static void elevator_release(struct kobject *kobj)
{
	struct elevator_queue *e;

	e = container_of(kobj, struct elevator_queue, kobj);
	elevator_put(e->type);
	kfree(e);
}

int elevator_init(struct request_queue *q, char *name)
{
	struct elevator_type *e = NULL;
	int err;

	/*
	 * q->sysfs_lock must be held to provide mutual exclusion between
	 * elevator_switch() and here.
	 */
	lockdep_assert_held(&q->sysfs_lock);

	if (unlikely(q->elevator))//yyf: 首先需要有elevator_queue
		return 0;

	INIT_LIST_HEAD(&q->queue_head);
	q->last_merge = NULL;
	q->end_sector = 0;
	q->boundary_rq = NULL;

	if (name) {
		e = elevator_get(name, true);//yyf: 如果有name参数，则查找该名字的elevator_type，找不到就返回错误
		if (!e)
			return -EINVAL;
	}

	/*
	 * Use the default elevator specified by config boot param for
	 * non-mq devices, or by config option. Don't try to load modules
	 * as we could be running off async and request_module() isn't
	 * allowed from async.
	 */
	if (!e && !q->mq_ops && *chosen_elevator) {//yyf: 如果启动参数有设置选择的elevator，则查找
		e = elevator_get(chosen_elevator, false);
		if (!e)
			printk(KERN_ERR "I/O scheduler %s not found\n",
							chosen_elevator);
	}

	if (!e) {
		/*
		 * For blk-mq devices, we default to using mq-deadline,
		 * if available, for single queue devices. If deadline
		 * isn't available OR we have multiple queues, default
		 * to "none".
		 */
		if (q->mq_ops) {
			if (q->nr_hw_queues == 1)
				e = elevator_get("mq-deadline", false);//yyf: 对于mq，hw_queue=1时，默认选择mq-deadline
			if (!e)
				return 0;
		} else
			e = elevator_get(CONFIG_DEFAULT_IOSCHED, false);//yyf: 前面都找不到的情况下，则查找config配置的算法

		if (!e) {
			printk(KERN_ERR
				"Default I/O scheduler not found. " \
				"Using noop.\n");
			e = elevator_get("noop", false);//yyf: 如果还是都找不到，则默认用noop电梯调度
		}
	}

	if (e->uses_mq)
		err = blk_mq_init_sched(q, e);
	else
		err = e->ops.sq.elevator_init_fn(q, e);//yyf: 调用elevator ops的elevator_init_fn初始化
	if (err)
		elevator_put(e);
	return err;
}
EXPORT_SYMBOL(elevator_init);

void elevator_exit(struct request_queue *q, struct elevator_queue *e)
{
	mutex_lock(&e->sysfs_lock);
	if (e->uses_mq && e->type->ops.mq.exit_sched)
		blk_mq_exit_sched(q, e);
	else if (!e->uses_mq && e->type->ops.sq.elevator_exit_fn)//yyf: 退出则只需elevator ops的elevator_exit_fn钩子函数
		e->type->ops.sq.elevator_exit_fn(e);
	mutex_unlock(&e->sysfs_lock);

	kobject_put(&e->kobj);
}
EXPORT_SYMBOL(elevator_exit);

static inline void __elv_rqhash_del(struct request *rq)
{
	hash_del(&rq->hash);
	rq->rq_flags &= ~RQF_HASHED;
}

void elv_rqhash_del(struct request_queue *q, struct request *rq)
{
	if (ELV_ON_HASH(rq)) //yyf: rq_flags & RQF_HASHED 标记
		__elv_rqhash_del(rq);
}
EXPORT_SYMBOL_GPL(elv_rqhash_del);

void elv_rqhash_add(struct request_queue *q, struct request *rq)
{
	struct elevator_queue *e = q->elevator;

	BUG_ON(ELV_ON_HASH(rq));
	hash_add(e->hash, &rq->hash, rq_hash_key(rq)); //yyf: 将rq hash节点加入到elevator_queue的hash数组
	rq->rq_flags |= RQF_HASHED; //yyf: 设置rq->rq_flags的RQF_HASHED标记位
}
EXPORT_SYMBOL_GPL(elv_rqhash_add);

void elv_rqhash_reposition(struct request_queue *q, struct request *rq)
{
	__elv_rqhash_del(rq); //yyf: 从原有的hash中删除
	elv_rqhash_add(q, rq); //yyf: 重新加入hash数组链表
}

struct request *elv_rqhash_find(struct request_queue *q, sector_t offset)
{
	struct elevator_queue *e = q->elevator;
	struct hlist_node *next;
	struct request *rq;

	hash_for_each_possible_safe(e->hash, rq, next, hash, offset) {
		BUG_ON(!ELV_ON_HASH(rq));

		if (unlikely(!rq_mergeable(rq))) { //yyf: 如果rq不能merge，则将rq从elevator_queue的hash数组链表删除
			__elv_rqhash_del(rq);
			continue;
		}

		if (rq_hash_key(rq) == offset)//rq的结束扇区和offset扇区相等，则找到了该rq
			return rq;
	}

	return NULL;
}

/*
 * RB-tree support functions for inserting/lookup/removal of requests
 * in a sorted RB tree.
 */
void elv_rb_add(struct rb_root *root, struct request *rq)
{
	struct rb_node **p = &root->rb_node;
	struct rb_node *parent = NULL;
	struct request *__rq;

	while (*p) {
		parent = *p;
		__rq = rb_entry(parent, struct request, rb_node);
        
//yyf: 红黑树 按rq的扇区号排序
		if (blk_rq_pos(rq) < blk_rq_pos(__rq))
			p = &(*p)->rb_left;
		else if (blk_rq_pos(rq) >= blk_rq_pos(__rq))
			p = &(*p)->rb_right;
	}
    
//yyf: 将rq插入到红黑树中
	rb_link_node(&rq->rb_node, parent, p);
	rb_insert_color(&rq->rb_node, root);
}
EXPORT_SYMBOL(elv_rb_add);

void elv_rb_del(struct rb_root *root, struct request *rq)
{
	BUG_ON(RB_EMPTY_NODE(&rq->rb_node));
	rb_erase(&rq->rb_node, root);//yyf: 从红黑树中删除rq
	RB_CLEAR_NODE(&rq->rb_node);
}
EXPORT_SYMBOL(elv_rb_del);

//yyf: 红黑树中查找sector对应的rq
struct request *elv_rb_find(struct rb_root *root, sector_t sector)
{
	struct rb_node *n = root->rb_node;
	struct request *rq;

	while (n) {
		rq = rb_entry(n, struct request, rb_node);

		if (sector < blk_rq_pos(rq))
			n = n->rb_left;
		else if (sector > blk_rq_pos(rq))
			n = n->rb_right;
		else
			return rq; //yyf: 如果rq的扇区等于sector，则找到
	}

	return NULL;
}
EXPORT_SYMBOL(elv_rb_find);

/*
 * Insert rq into dispatch queue of q.  Queue lock must be held on
 * entry.  rq is sort instead into the dispatch queue. To be used by
 * specific elevators.
 */
void elv_dispatch_sort(struct request_queue *q, struct request *rq)
{
	sector_t boundary;
	struct list_head *entry;

	if (q->last_merge == rq)
		q->last_merge = NULL;

	elv_rqhash_del(q, rq);//yyf: 先从hash链表删除

	q->nr_sorted--;

	boundary = q->end_sector;
	list_for_each_prev(entry, &q->queue_head) { //yyf: 遍历q的queuelist链表
		struct request *pos = list_entry_rq(entry);

		if (req_op(rq) != req_op(pos))//yyf: 如果请求操作不一样，则跳出
			break;
		if (rq_data_dir(rq) != rq_data_dir(pos)) //yyf: 读写操作不一样，则跳出
			break;
		if (pos->rq_flags & (RQF_STARTED | RQF_SOFTBARRIER))//yyf: RQF_STARTED | RQF_SOFTBARRIER如果是这2种标记，则跳出
			break;
		if (blk_rq_pos(rq) >= boundary) {
			if (blk_rq_pos(pos) < boundary)
				continue;
		} else {
			if (blk_rq_pos(pos) >= boundary)
				break;
		}
		if (blk_rq_pos(rq) >= blk_rq_pos(pos)) //yyf: 如果rq的扇区大于等于当前pos请求位置
			break;
	}

	list_add(&rq->queuelist, entry);//yyf: 挂入到queuelist链表
}
EXPORT_SYMBOL(elv_dispatch_sort);

/*
 * Insert rq into dispatch queue of q.  Queue lock must be held on
 * entry.  rq is added to the back of the dispatch queue. To be used by
 * specific elevators.
 */
void elv_dispatch_add_tail(struct request_queue *q, struct request *rq)
{
	if (q->last_merge == rq)
		q->last_merge = NULL;

	elv_rqhash_del(q, rq); //yyf: 从hash数组链表删除

	q->nr_sorted--;

	q->end_sector = rq_end_sector(rq); //yyf: q最大扇区
	q->boundary_rq = rq; //yyf:最大扇区对应的边界rq
	list_add_tail(&rq->queuelist, &q->queue_head); //yyf: 挂入到q->queue_head队列尾
}
EXPORT_SYMBOL(elv_dispatch_add_tail);

//yyf: 这个是合并bio到rq的
enum elv_merge elv_merge(struct request_queue *q, struct request **req,
		struct bio *bio)
{
	struct elevator_queue *e = q->elevator;
	struct request *__rq;

	/*
	 * Levels of merges:
	 * 	nomerges:  No merges at all attempted
	 * 	noxmerges: Only simple one-hit cache try
	 * 	merges:	   All merge tries attempted
	 */
	if (blk_queue_nomerges(q) || !bio_mergeable(bio))
		return ELEVATOR_NO_MERGE;

//yyf: bio和q->last_merge缓存的rq做一次merge
	/*
	 * First try one-hit cache.
	 */
	if (q->last_merge && elv_bio_merge_ok(q->last_merge, bio)) {
		enum elv_merge ret = blk_try_merge(q->last_merge, bio);

		if (ret != ELEVATOR_NO_MERGE) {
			*req = q->last_merge;
			return ret;
		}
	}
//yyf: noxmerges就是和前面的q->last_merge缓存进行merge的一次机会，如果前面merge不了，则返回
	if (blk_queue_noxmerges(q))
		return ELEVATOR_NO_MERGE;

//yyf: 从hash数组链表中找对应sector可back merge的rq，elv_rqhash_find里面是判断rq的扇区结束是否和bio sector相等
	/*
	 * See if our hash lookup can find a potential backmerge.
	 */
	__rq = elv_rqhash_find(q, bio->bi_iter.bi_sector);
	if (__rq && elv_bio_merge_ok(__rq, bio)) { //yyf: 如果找到rq，则说明扇区和bio是相连的，再判断是否具备可merge的其他条件
		*req = __rq;
		return ELEVATOR_BACK_MERGE;
	}
//yyf: 最后再看看elevator里的merge相关 钩子接口
	if (e->uses_mq && e->type->ops.mq.request_merge)
		return e->type->ops.mq.request_merge(q, req, bio);
	else if (!e->uses_mq && e->type->ops.sq.elevator_merge_fn)
		return e->type->ops.sq.elevator_merge_fn(q, req, bio);

	return ELEVATOR_NO_MERGE;
}

/*
 * Attempt to do an insertion back merge. Only check for the case where
 * we can append 'rq' to an existing request, so we can throw 'rq' away
 * afterwards.
 *
 * Returns true if we merged, false otherwise
 */
bool elv_attempt_insert_merge(struct request_queue *q, struct request *rq)
{
	struct request *__rq;
	bool ret;

	if (blk_queue_nomerges(q))//yyf: 判断request_queue是否就不可merge
		return false;

	/*
	 * First try one-hit cache.
	 */
	if (q->last_merge && blk_attempt_req_merge(q, q->last_merge, rq))
		return true;

	if (blk_queue_noxmerges(q))//yyf: request_queue noxmerges判断
		return false;

	ret = false;
	/*
	 * See if our hash lookup can find a potential backmerge.
	 */
	while (1) {
		__rq = elv_rqhash_find(q, blk_rq_pos(rq));
		if (!__rq || !blk_attempt_req_merge(q, __rq, rq))
			break;

		/* The merged request could be merged with others, try again */
		ret = true;
		rq = __rq;
	}

	return ret;
}

void elv_merged_request(struct request_queue *q, struct request *rq,
		enum elv_merge type)
{
	struct elevator_queue *e = q->elevator;

	if (e->uses_mq && e->type->ops.mq.request_merged)
		e->type->ops.mq.request_merged(q, rq, type);
	else if (!e->uses_mq && e->type->ops.sq.elevator_merged_fn)
		e->type->ops.sq.elevator_merged_fn(q, rq, type);

	if (type == ELEVATOR_BACK_MERGE)
		elv_rqhash_reposition(q, rq);

	q->last_merge = rq;
}

void elv_merge_requests(struct request_queue *q, struct request *rq,
			     struct request *next)
{
	struct elevator_queue *e = q->elevator;
	bool next_sorted = false;

	if (e->uses_mq && e->type->ops.mq.requests_merged)
		e->type->ops.mq.requests_merged(q, rq, next);
	else if (e->type->ops.sq.elevator_merge_req_fn) {
		next_sorted = (__force bool)(next->rq_flags & RQF_SORTED);
		if (next_sorted)
			e->type->ops.sq.elevator_merge_req_fn(q, rq, next);
	}

	elv_rqhash_reposition(q, rq); //yyf: 重新调整下hash链表

	if (next_sorted) {
		elv_rqhash_del(q, next);
		q->nr_sorted--;
	}

	q->last_merge = rq;
}

void elv_bio_merged(struct request_queue *q, struct request *rq,
			struct bio *bio)
{
	struct elevator_queue *e = q->elevator;

	if (WARN_ON_ONCE(e->uses_mq))
		return;

	if (e->type->ops.sq.elevator_bio_merged_fn)
		e->type->ops.sq.elevator_bio_merged_fn(q, rq, bio);
}

#ifdef CONFIG_PM
static void blk_pm_requeue_request(struct request *rq)
{
	if (rq->q->dev && !(rq->rq_flags & RQF_PM))
		rq->q->nr_pending--;
}

static void blk_pm_add_request(struct request_queue *q, struct request *rq)
{
	if (q->dev && !(rq->rq_flags & RQF_PM) && q->nr_pending++ == 0 &&
	    (q->rpm_status == RPM_SUSPENDED || q->rpm_status == RPM_SUSPENDING))
		pm_request_resume(q->dev);
}
#else
static inline void blk_pm_requeue_request(struct request *rq) {}
static inline void blk_pm_add_request(struct request_queue *q,
				      struct request *rq)
{
}
#endif

void elv_requeue_request(struct request_queue *q, struct request *rq)
{
	/*
	 * it already went through dequeue, we need to decrement the
	 * in_flight count again
	 */
	if (blk_account_rq(rq)) {
		q->in_flight[rq_is_sync(rq)]--;
		if (rq->rq_flags & RQF_SORTED)
			elv_deactivate_rq(q, rq);
	}

	rq->rq_flags &= ~RQF_STARTED;

	blk_pm_requeue_request(rq);

	__elv_add_request(q, rq, ELEVATOR_INSERT_REQUEUE);
}

void elv_drain_elevator(struct request_queue *q)
{
	struct elevator_queue *e = q->elevator;
	static int printed;

	if (WARN_ON_ONCE(e->uses_mq))
		return;

	lockdep_assert_held(q->queue_lock);

	while (e->type->ops.sq.elevator_dispatch_fn(q, 1))
		;
	if (q->nr_sorted && printed++ < 10) {
		printk(KERN_ERR "%s: forced dispatching is broken "
		       "(nr_sorted=%u), please report this\n",
		       q->elevator->type->elevator_name, q->nr_sorted);
	}
}

void __elv_add_request(struct request_queue *q, struct request *rq, int where)
{
	trace_block_rq_insert(q, rq);

	blk_pm_add_request(q, rq);

	rq->q = q;

	if (rq->rq_flags & RQF_SOFTBARRIER) {
		/* barriers are scheduling boundary, update end_sector */
		if (!blk_rq_is_passthrough(rq)) {
			q->end_sector = rq_end_sector(rq);
			q->boundary_rq = rq;
		}
	} else if (!(rq->rq_flags & RQF_ELVPRIV) &&
		    (where == ELEVATOR_INSERT_SORT ||
		     where == ELEVATOR_INSERT_SORT_MERGE))
		where = ELEVATOR_INSERT_BACK;

	switch (where) {
	case ELEVATOR_INSERT_REQUEUE:
	case ELEVATOR_INSERT_FRONT:
		rq->rq_flags |= RQF_SOFTBARRIER;
		list_add(&rq->queuelist, &q->queue_head);
		break;

	case ELEVATOR_INSERT_BACK:
		rq->rq_flags |= RQF_SOFTBARRIER;
		elv_drain_elevator(q);
		list_add_tail(&rq->queuelist, &q->queue_head);
		/*
		 * We kick the queue here for the following reasons.
		 * - The elevator might have returned NULL previously
		 *   to delay requests and returned them now.  As the
		 *   queue wasn't empty before this request, ll_rw_blk
		 *   won't run the queue on return, resulting in hang.
		 * - Usually, back inserted requests won't be merged
		 *   with anything.  There's no point in delaying queue
		 *   processing.
		 */
		__blk_run_queue(q);
		break;

	case ELEVATOR_INSERT_SORT_MERGE:
		/*
		 * If we succeed in merging this request with one in the
		 * queue already, we are done - rq has now been freed,
		 * so no need to do anything further.
		 */
		if (elv_attempt_insert_merge(q, rq))
			break;
	case ELEVATOR_INSERT_SORT:
		BUG_ON(blk_rq_is_passthrough(rq));
		rq->rq_flags |= RQF_SORTED;
		q->nr_sorted++;
		if (rq_mergeable(rq)) {
			elv_rqhash_add(q, rq);
			if (!q->last_merge)
				q->last_merge = rq;
		}

		/*
		 * Some ioscheds (cfq) run q->request_fn directly, so
		 * rq cannot be accessed after calling
		 * elevator_add_req_fn.
		 */
		q->elevator->type->ops.sq.elevator_add_req_fn(q, rq);
		break;

	case ELEVATOR_INSERT_FLUSH:
		rq->rq_flags |= RQF_SOFTBARRIER;
		blk_insert_flush(rq);
		break;
	default:
		printk(KERN_ERR "%s: bad insertion point %d\n",
		       __func__, where);
		BUG();
	}
}
EXPORT_SYMBOL(__elv_add_request);

void elv_add_request(struct request_queue *q, struct request *rq, int where)
{
	unsigned long flags;

	spin_lock_irqsave(q->queue_lock, flags);
	__elv_add_request(q, rq, where);
	spin_unlock_irqrestore(q->queue_lock, flags);
}
EXPORT_SYMBOL(elv_add_request);

struct request *elv_latter_request(struct request_queue *q, struct request *rq)
{
	struct elevator_queue *e = q->elevator;

	if (e->uses_mq && e->type->ops.mq.next_request)
		return e->type->ops.mq.next_request(q, rq);
	else if (!e->uses_mq && e->type->ops.sq.elevator_latter_req_fn)
		return e->type->ops.sq.elevator_latter_req_fn(q, rq);

	return NULL;
}

struct request *elv_former_request(struct request_queue *q, struct request *rq)
{
	struct elevator_queue *e = q->elevator;

	if (e->uses_mq && e->type->ops.mq.former_request)
		return e->type->ops.mq.former_request(q, rq);
	if (!e->uses_mq && e->type->ops.sq.elevator_former_req_fn)
		return e->type->ops.sq.elevator_former_req_fn(q, rq);
	return NULL;
}

int elv_set_request(struct request_queue *q, struct request *rq,
		    struct bio *bio, gfp_t gfp_mask)
{
	struct elevator_queue *e = q->elevator;

	if (WARN_ON_ONCE(e->uses_mq))
		return 0;

	if (e->type->ops.sq.elevator_set_req_fn)
		return e->type->ops.sq.elevator_set_req_fn(q, rq, bio, gfp_mask);
	return 0;
}

void elv_put_request(struct request_queue *q, struct request *rq)
{
	struct elevator_queue *e = q->elevator;

	if (WARN_ON_ONCE(e->uses_mq))
		return;

	if (e->type->ops.sq.elevator_put_req_fn)
		e->type->ops.sq.elevator_put_req_fn(rq);
}

int elv_may_queue(struct request_queue *q, unsigned int op)
{
	struct elevator_queue *e = q->elevator;

	if (WARN_ON_ONCE(e->uses_mq))
		return 0;

	if (e->type->ops.sq.elevator_may_queue_fn)
		return e->type->ops.sq.elevator_may_queue_fn(q, op);

	return ELV_MQUEUE_MAY;
}

void elv_completed_request(struct request_queue *q, struct request *rq)
{
	struct elevator_queue *e = q->elevator;

	if (WARN_ON_ONCE(e->uses_mq))
		return;

	/*
	 * request is released from the driver, io must be done
	 */
	if (blk_account_rq(rq)) {
		q->in_flight[rq_is_sync(rq)]--;
		if ((rq->rq_flags & RQF_SORTED) &&
		    e->type->ops.sq.elevator_completed_req_fn)
			e->type->ops.sq.elevator_completed_req_fn(q, rq);
	}
}

#define to_elv(atr) container_of((atr), struct elv_fs_entry, attr)

static ssize_t
elv_attr_show(struct kobject *kobj, struct attribute *attr, char *page)
{
	struct elv_fs_entry *entry = to_elv(attr);
	struct elevator_queue *e;
	ssize_t error;

	if (!entry->show)
		return -EIO;

	e = container_of(kobj, struct elevator_queue, kobj);
	mutex_lock(&e->sysfs_lock);
	error = e->type ? entry->show(e, page) : -ENOENT;
	mutex_unlock(&e->sysfs_lock);
	return error;
}

static ssize_t
elv_attr_store(struct kobject *kobj, struct attribute *attr,
	       const char *page, size_t length)
{
	struct elv_fs_entry *entry = to_elv(attr);
	struct elevator_queue *e;
	ssize_t error;

	if (!entry->store)
		return -EIO;

	e = container_of(kobj, struct elevator_queue, kobj);
	mutex_lock(&e->sysfs_lock);
	error = e->type ? entry->store(e, page, length) : -ENOENT;
	mutex_unlock(&e->sysfs_lock);
	return error;
}

static const struct sysfs_ops elv_sysfs_ops = {
	.show	= elv_attr_show,
	.store	= elv_attr_store,
};

static struct kobj_type elv_ktype = {
	.sysfs_ops	= &elv_sysfs_ops,
	.release	= elevator_release,
};

int elv_register_queue(struct request_queue *q)
{
	struct elevator_queue *e = q->elevator;
	int error;

	error = kobject_add(&e->kobj, &q->kobj, "%s", "iosched");//yyf: 在sys下request_queue目录注册elevator_queue，目录名为iosched
	if (!error) {
		struct elv_fs_entry *attr = e->type->elevator_attrs;
		if (attr) {
			while (attr->attr.name) {
				if (sysfs_create_file(&e->kobj, &attr->attr))
					break;
				attr++;
			}
		}
		kobject_uevent(&e->kobj, KOBJ_ADD);
		e->registered = 1;
		if (!e->uses_mq && e->type->ops.sq.elevator_registered_fn)
			e->type->ops.sq.elevator_registered_fn(q);
	}
	return error;
}
EXPORT_SYMBOL(elv_register_queue);

void elv_unregister_queue(struct request_queue *q)
{
	if (q) {
		struct elevator_queue *e = q->elevator;

		kobject_uevent(&e->kobj, KOBJ_REMOVE);
		kobject_del(&e->kobj);
		e->registered = 0;
		/* Re-enable throttling in case elevator disabled it */
		wbt_enable_default(q);
	}
}
EXPORT_SYMBOL(elv_unregister_queue);

int elv_register(struct elevator_type *e)
{
	char *def = "";

	/* create icq_cache if requested */
	if (e->icq_size) {
		if (WARN_ON(e->icq_size < sizeof(struct io_cq)) ||
		    WARN_ON(e->icq_align < __alignof__(struct io_cq)))
			return -EINVAL;

		snprintf(e->icq_cache_name, sizeof(e->icq_cache_name),
			 "%s_io_cq", e->elevator_name);
		e->icq_cache = kmem_cache_create(e->icq_cache_name, e->icq_size,
						 e->icq_align, 0, NULL);
		if (!e->icq_cache)
			return -ENOMEM;
	}

	/* register, don't allow duplicate names */
	spin_lock(&elv_list_lock);
	if (elevator_find(e->elevator_name)) {
		spin_unlock(&elv_list_lock);
		if (e->icq_cache)
			kmem_cache_destroy(e->icq_cache);
		return -EBUSY;
	}
	list_add_tail(&e->list, &elv_list);
	spin_unlock(&elv_list_lock);

	/* print pretty message */
	if (!strcmp(e->elevator_name, chosen_elevator) ||
			(!*chosen_elevator &&
			 !strcmp(e->elevator_name, CONFIG_DEFAULT_IOSCHED)))
				def = " (default)";

	printk(KERN_INFO "io scheduler %s registered%s\n", e->elevator_name,
								def);
	return 0;
}
EXPORT_SYMBOL_GPL(elv_register);

void elv_unregister(struct elevator_type *e)
{
	/* unregister */
	spin_lock(&elv_list_lock);
	list_del_init(&e->list);
	spin_unlock(&elv_list_lock);

	/*
	 * Destroy icq_cache if it exists.  icq's are RCU managed.  Make
	 * sure all RCU operations are complete before proceeding.
	 */
	if (e->icq_cache) {
		rcu_barrier();
		kmem_cache_destroy(e->icq_cache);
		e->icq_cache = NULL;
	}
}
EXPORT_SYMBOL_GPL(elv_unregister);

static int elevator_switch_mq(struct request_queue *q,
			      struct elevator_type *new_e)
{
	int ret;

	blk_mq_freeze_queue(q);

	if (q->elevator) {
		if (q->elevator->registered)
			elv_unregister_queue(q);
		ioc_clear_queue(q);
		elevator_exit(q, q->elevator);
	}

	ret = blk_mq_init_sched(q, new_e);
	if (ret)
		goto out;

	if (new_e) {
		ret = elv_register_queue(q);
		if (ret) {
			elevator_exit(q, q->elevator);
			goto out;
		}
	}

	if (new_e)
		blk_add_trace_msg(q, "elv switch: %s", new_e->elevator_name);
	else
		blk_add_trace_msg(q, "elv switch: none");

out:
	blk_mq_unfreeze_queue(q);
	return ret;
}

/*
 * switch to new_e io scheduler. be careful not to introduce deadlocks -
 * we don't free the old io scheduler, before we have allocated what we
 * need for the new one. this way we have a chance of going back to the old
 * one, if the new one fails init for some reason.
 */
static int elevator_switch(struct request_queue *q, struct elevator_type *new_e)
{
	struct elevator_queue *old = q->elevator;
	bool old_registered = false;
	int err;

	if (q->mq_ops)
		return elevator_switch_mq(q, new_e);

	/*
	 * Turn on BYPASS and drain all requests w/ elevator private data.
	 * Block layer doesn't call into a quiesced elevator - all requests
	 * are directly put on the dispatch list without elevator data
	 * using INSERT_BACK.  All requests have SOFTBARRIER set and no
	 * merge happens either.
	 */
	if (old) {
		old_registered = old->registered;

		blk_queue_bypass_start(q);

		/* unregister and clear all auxiliary data of the old elevator */
		if (old_registered)
			elv_unregister_queue(q);

		ioc_clear_queue(q);
	}

	/* allocate, init and register new elevator */
	err = new_e->ops.sq.elevator_init_fn(q, new_e);
	if (err)
		goto fail_init;

	err = elv_register_queue(q);
	if (err)
		goto fail_register;

	/* done, kill the old one and finish */
	if (old) {
		elevator_exit(q, old);
		blk_queue_bypass_end(q);
	}

	blk_add_trace_msg(q, "elv switch: %s", new_e->elevator_name);

	return 0;

fail_register:
	elevator_exit(q, q->elevator);
fail_init:
	/* switch failed, restore and re-register old elevator */
	if (old) {
		q->elevator = old;
		elv_register_queue(q);
		blk_queue_bypass_end(q);
	}

	return err;
}

/*
 * Switch this queue to the given IO scheduler.
 */
static int __elevator_change(struct request_queue *q, const char *name)
{
	char elevator_name[ELV_NAME_MAX];
	struct elevator_type *e;

	/*
	 * Special case for mq, turn off scheduling
	 */
	if (q->mq_ops && !strncmp(name, "none", 4))
		return elevator_switch(q, NULL);

	strlcpy(elevator_name, name, sizeof(elevator_name));
	e = elevator_get(strstrip(elevator_name), true);
	if (!e)
		return -EINVAL;

	if (q->elevator &&
	    !strcmp(elevator_name, q->elevator->type->elevator_name)) {
		elevator_put(e);
		return 0;
	}

	if (!e->uses_mq && q->mq_ops) {
		elevator_put(e);
		return -EINVAL;
	}
	if (e->uses_mq && !q->mq_ops) {
		elevator_put(e);
		return -EINVAL;
	}

	return elevator_switch(q, e);
}

static inline bool elv_support_iosched(struct request_queue *q)
{
	if (q->mq_ops && q->tag_set && (q->tag_set->flags &
				BLK_MQ_F_NO_SCHED)) //yyf: 对于mq，则有不支持iosched调度算法的情况
		return false;
	return true;
}

ssize_t elv_iosched_store(struct request_queue *q, const char *name,
			  size_t count)
{
	int ret;
    
//yyf: 如果是mq的mq_ops存在，或者q->request_fn存在，或者q不支持调度算法，则不能通过proc设置elevator
	if (!(q->mq_ops || q->request_fn) || !elv_support_iosched(q))
		return count;

	ret = __elevator_change(q, name);
	if (!ret)
		return count;

	return ret;
}

ssize_t elv_iosched_show(struct request_queue *q, char *name)
{
	struct elevator_queue *e = q->elevator;
	struct elevator_type *elv = NULL;
	struct elevator_type *__e;
	int len = 0;

	if (!blk_queue_stackable(q))
		return sprintf(name, "none\n");

	if (!q->elevator)
		len += sprintf(name+len, "[none] ");
	else
		elv = e->type;

	spin_lock(&elv_list_lock);
	list_for_each_entry(__e, &elv_list, list) {
		if (elv && !strcmp(elv->elevator_name, __e->elevator_name)) {
			len += sprintf(name+len, "[%s] ", elv->elevator_name);
			continue;
		}
		if (__e->uses_mq && q->mq_ops && elv_support_iosched(q))
			len += sprintf(name+len, "%s ", __e->elevator_name);
		else if (!__e->uses_mq && !q->mq_ops)
			len += sprintf(name+len, "%s ", __e->elevator_name);
	}
	spin_unlock(&elv_list_lock);

	if (q->mq_ops && q->elevator)
		len += sprintf(name+len, "none");

	len += sprintf(len+name, "\n");
	return len;
}

//yyf: rq红黑树节点的前一个
struct request *elv_rb_former_request(struct request_queue *q,
				      struct request *rq)
{
	struct rb_node *rbprev = rb_prev(&rq->rb_node);

	if (rbprev)
		return rb_entry_rq(rbprev);

	return NULL;
}
EXPORT_SYMBOL(elv_rb_former_request);

//yyf: rq红黑树节点的后一个
struct request *elv_rb_latter_request(struct request_queue *q,
				      struct request *rq)
{
	struct rb_node *rbnext = rb_next(&rq->rb_node);

	if (rbnext)
		return rb_entry_rq(rbnext);

	return NULL;
}
EXPORT_SYMBOL(elv_rb_latter_request);
