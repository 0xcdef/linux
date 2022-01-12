// SPDX-License-Identifier: GPL-2.0-only
/*
 * SMP initialisation and IPI support
 * Based on arch/arm64/kernel/smp.c
 *
 * Copyright (C) 2012 ARM Ltd.
 * Copyright (C) 2015 Regents of the University of California
 * Copyright (C) 2017 SiFive
 */

#include <linux/cpu.h>
#include <linux/clockchips.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/profile.h>
#include <linux/smp.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/irq_work.h>

#include <asm/tlbflush.h>
#include <asm/cacheflush.h>

enum ipi_message_type {
	IPI_RESCHEDULE,
	IPI_CALL_FUNC,
	IPI_CPU_STOP,
	IPI_IRQ_WORK,
	IPI_TIMER,
	IPI_MAX
};

unsigned long __cpuid_to_hartid_map[NR_CPUS] __ro_after_init = {
	[0 ... NR_CPUS-1] = INVALID_HARTID
};

void __init smp_setup_processor_id(void)
{
	cpuid_to_hartid_map(0) = boot_cpu_hartid;
}

static int ipi_virq_base __ro_after_init;
static int nr_ipi __ro_after_init = IPI_MAX;
static struct irq_desc *ipi_desc[IPI_MAX] __read_mostly;

int riscv_hartid_to_cpuid(int hartid)
{
	int i;

	for (i = 0; i < NR_CPUS; i++)
		if (cpuid_to_hartid_map(i) == hartid)
			return i;

	pr_err("Couldn't find cpu id for hartid [%d]\n", hartid);
	return -ENOENT;
}

void riscv_cpuid_to_hartid_mask(const struct cpumask *in, struct cpumask *out)
{
	int cpu;

	cpumask_clear(out);
	for_each_cpu(cpu, in)
		cpumask_set_cpu(cpuid_to_hartid_map(cpu), out);
}
EXPORT_SYMBOL_GPL(riscv_cpuid_to_hartid_mask);

bool arch_match_cpu_phys_id(int cpu, u64 phys_id)
{
	return phys_id == cpuid_to_hartid_map(cpu);
}

/* Unsupported */
int setup_profiling_timer(unsigned int multiplier)
{
	return -EINVAL;
}

static void ipi_stop(void)
{
	set_cpu_online(smp_processor_id(), false);
	while (1)
		wait_for_interrupt();
}

static void send_ipi_mask(const struct cpumask *mask, enum ipi_message_type op)
{
	__ipi_send_mask(ipi_desc[op], mask);
}

static void send_ipi_single(int cpu, enum ipi_message_type op)
{
	__ipi_send_mask(ipi_desc[op], cpumask_of(cpu));
}

#ifdef CONFIG_IRQ_WORK
void arch_irq_work_raise(void)
{
	send_ipi_single(smp_processor_id(), IPI_IRQ_WORK);
}
#endif

static irqreturn_t handle_IPI(int irq, void *data)
{
	int ipi = irq - ipi_virq_base;

	switch (ipi) {
	case IPI_RESCHEDULE:
		scheduler_ipi();
		break;
	case IPI_CALL_FUNC:
		generic_smp_call_function_interrupt();
		break;
	case IPI_CPU_STOP:
		ipi_stop();
		break;
	case IPI_IRQ_WORK:
		irq_work_run();
		break;
#ifdef CONFIG_GENERIC_CLOCKEVENTS_BROADCAST
	case IPI_TIMER:
		tick_receive_broadcast();
		break;
#endif
	default:
		pr_warn("CPU%d: unhandled IPI%d\n", smp_processor_id(), ipi);
		break;
	};

	return IRQ_HANDLED;
}

void riscv_ipi_enable(void)
{
	int i;

	if (WARN_ON_ONCE(!ipi_virq_base))
		return;

	for (i = 0; i < nr_ipi; i++)
		enable_percpu_irq(ipi_virq_base + i, 0);
}

void riscv_ipi_disable(void)
{
	int i;

	if (WARN_ON_ONCE(!ipi_virq_base))
		return;

	for (i = 0; i < nr_ipi; i++)
		disable_percpu_irq(ipi_virq_base + i);
}

bool riscv_ipi_have_virq_range(void)
{
	return (ipi_virq_base) ? true : false;
}

void riscv_ipi_set_virq_range(int virq, int nr)
{
	int i, err;

	if (WARN_ON(ipi_virq_base))
		return;

	WARN_ON(nr < IPI_MAX);
	nr_ipi = min(nr, IPI_MAX);
	ipi_virq_base = virq;

	/* Request IPIs */
	for (i = 0; i < nr_ipi; i++) {
		err = request_percpu_irq(ipi_virq_base + i, handle_IPI,
					 "IPI", &ipi_virq_base);
		WARN_ON(err);

		ipi_desc[i] = irq_to_desc(ipi_virq_base + i);
		irq_set_status_flags(ipi_virq_base + i, IRQ_HIDDEN);
	}

	/* Enabled IPIs for boot CPU immediately */
	riscv_ipi_enable();
}
EXPORT_SYMBOL_GPL(riscv_ipi_set_virq_range);

static const char * const ipi_names[] = {
	[IPI_RESCHEDULE]	= "Rescheduling interrupts",
	[IPI_CALL_FUNC]		= "Function call interrupts",
	[IPI_CPU_STOP]		= "CPU stop interrupts",
	[IPI_IRQ_WORK]		= "IRQ work interrupts",
	[IPI_TIMER]		= "Timer broadcast interrupts",
};

void show_ipi_stats(struct seq_file *p, int prec)
{
	unsigned int cpu, i;

	for (i = 0; i < IPI_MAX; i++) {
		seq_printf(p, "%*s%u:%s", prec - 1, "IPI", i,
			   prec >= 4 ? " " : "");
		for_each_online_cpu(cpu)
			seq_printf(p, "%10u ", irq_desc_kstat_cpu(ipi_desc[i], cpu));
		seq_printf(p, " %s\n", ipi_names[i]);
	}
}

void arch_send_call_function_ipi_mask(struct cpumask *mask)
{
	send_ipi_mask(mask, IPI_CALL_FUNC);
}

void arch_send_call_function_single_ipi(int cpu)
{
	send_ipi_single(cpu, IPI_CALL_FUNC);
}

#ifdef CONFIG_GENERIC_CLOCKEVENTS_BROADCAST
void tick_broadcast(const struct cpumask *mask)
{
	send_ipi_mask(mask, IPI_TIMER);
}
#endif

void smp_send_stop(void)
{
	unsigned long timeout;

	if (num_online_cpus() > 1) {
		cpumask_t mask;

		cpumask_copy(&mask, cpu_online_mask);
		cpumask_clear_cpu(smp_processor_id(), &mask);

		if (system_state <= SYSTEM_RUNNING)
			pr_crit("SMP: stopping secondary CPUs\n");
		send_ipi_mask(&mask, IPI_CPU_STOP);
	}

	/* Wait up to one second for other CPUs to stop */
	timeout = USEC_PER_SEC;
	while (num_online_cpus() > 1 && timeout--)
		udelay(1);

	if (num_online_cpus() > 1)
		pr_warn("SMP: failed to stop secondary CPUs %*pbl\n",
			   cpumask_pr_args(cpu_online_mask));
}

void smp_send_reschedule(int cpu)
{
	send_ipi_single(cpu, IPI_RESCHEDULE);
}
EXPORT_SYMBOL_GPL(smp_send_reschedule);