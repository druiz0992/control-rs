use crate::physics::Energy;
use crate::physics::traits::State;
use plotters::prelude::*;

/// Generic plot for states over time
pub fn plot_states<S: State>(
    times: &[f64],
    states: &[S],
    filename: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    let root = BitMapBackend::new(filename, (800, 600)).into_drawing_area();
    root.fill(&WHITE)?;

    let (min_value, max_value) = states
        .iter()
        .flat_map(|s| s.as_vec())
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), val| {
            (min.min(val), max.max(val))
        });

    let mut chart = ChartBuilder::on(&root)
        .caption("State trajectory", ("sans-serif", 30))
        .margin(10)
        .x_label_area_size(40)
        .y_label_area_size(40)
        .build_cartesian_2d(times[0]..times[times.len() - 1], min_value..max_value)?;

    chart.configure_mesh().draw()?;

    let num_states = states[0].as_vec().len();
    let labels = <S as State>::labels();
    for i in 0..num_states {
        let series: Vec<(f64, f64)> = times
            .iter()
            .zip(states.iter())
            .map(|(&t, s)| (t, s.as_vec()[i]))
            .collect();

        let label = labels.get(i).cloned().ok_or("Label index out of bounds")?;

        chart
            .draw_series(LineSeries::new(series, &Palette99::pick(i)))?
            .label(label)
            .legend(move |(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], Palette99::pick(i)));
    }

    chart
        .configure_series_labels()
        .background_style(WHITE)
        .draw()?;

    Ok(())
}

/// Plot total, kinetic, and potential energy over time
pub fn plot_energy(
    times: &[f64],
    energies: &[Energy],
    filename: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    let root = BitMapBackend::new(filename, (800, 600)).into_drawing_area();
    root.fill(&WHITE)?;

    let (_min_value, max_value) = energies
        .iter()
        .map(|s| s.total())
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), val| {
            (min.min(val), max.max(val))
        });
    let mut chart = ChartBuilder::on(&root)
        .caption("Energy Over Time", ("sans-serif", 30))
        .margin(10)
        .x_label_area_size(40)
        .y_label_area_size(40)
        .build_cartesian_2d(
            times[0]..times[times.len() - 1],
            0.0..max_value + max_value * 0.2,
        )?;

    chart.configure_mesh().draw()?;

    let mut plot_series = |extract: fn(&Energy) -> f64, color, label| {
        let series: Vec<(f64, f64)> = times
            .iter()
            .zip(energies.iter())
            .map(|(&t, e)| (t, extract(e)))
            .collect();
        chart
            .draw_series(LineSeries::new(series, color))?
            .label(label)
            .legend(move |(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], color));
        Ok::<_, Box<dyn std::error::Error>>(())
    };

    plot_series(|e| e.get_kinetic(), &RED, "Kinetic")?;
    plot_series(|e| e.get_potential(), &BLUE, "Potential")?;
    plot_series(|e| e.total(), &GREEN, "Total")?;

    chart
        .configure_series_labels()
        .background_style(WHITE)
        .draw()?;

    Ok(())
}
