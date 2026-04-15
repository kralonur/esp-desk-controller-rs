use alloc::{format, string::String};

use embassy_net::Stack;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use picoserve::{
    AppBuilder, AppRouter, ResponseSent, Router, routing,
    routing::{RequestHandlerService, parse_path_segment},
};

use picoserve::io::Read;
use picoserve::request::Request;
use picoserve::response::{IntoResponse, ResponseWriter};

#[derive(Clone, Copy)]
pub enum DeskCommand {
    Home,
    MoveTo(i32),
    MoveBy(i32),
}

pub struct DeskControllerState {
    commands: Channel<CriticalSectionRawMutex, DeskCommand, 1>,
}

impl DeskControllerState {
    pub const fn new() -> Self {
        Self {
            commands: Channel::new(),
        }
    }

    pub fn try_queue(&self, command: DeskCommand) -> bool {
        self.commands.try_send(command).is_ok()
    }

    pub async fn next_command(&self) -> DeskCommand {
        self.commands.receive().await
    }
}

async fn index() -> &'static str {
    "Endpoints: POST /home, /up/<steps>, /down/<steps>, /move/<position>\n"
}

fn home_response(state: &DeskControllerState) -> &'static str {
    if state.try_queue(DeskCommand::Home) {
        "home command queued\n"
    } else {
        "home command queue is full\n"
    }
}

fn up_response(state: &DeskControllerState, steps: i32) -> String {
    if state.try_queue(DeskCommand::MoveBy(steps.abs())) {
        format!("up command queued by {}\n", steps.abs())
    } else {
        String::from("command queue is full\n")
    }
}

fn down_response(state: &DeskControllerState, steps: i32) -> String {
    if state.try_queue(DeskCommand::MoveBy(-steps.abs())) {
        format!("down command queued by {}\n", steps.abs())
    } else {
        String::from("command queue is full\n")
    }
}

fn move_to_response(state: &DeskControllerState, position: i32) -> String {
    if state.try_queue(DeskCommand::MoveTo(position)) {
        format!("move command queued to {}\n", position)
    } else {
        String::from("command queue is full\n")
    }
}

#[derive(Clone, Copy)]
struct HomeService {
    state: &'static DeskControllerState,
}

#[derive(Clone, Copy)]
struct UpService {
    state: &'static DeskControllerState,
}

#[derive(Clone, Copy)]
struct DownService {
    state: &'static DeskControllerState,
}

#[derive(Clone, Copy)]
struct MoveToService {
    state: &'static DeskControllerState,
}

impl RequestHandlerService<(), ()> for HomeService {
    async fn call_request_handler_service<R: Read, W: ResponseWriter<Error = R::Error>>(
        &self,
        _state: &(),
        (): (),
        request: Request<'_, R>,
        response_writer: W,
    ) -> Result<ResponseSent, W::Error> {
        home_response(self.state)
            .write_to(request.body_connection.finalize().await?, response_writer)
            .await
    }
}

impl RequestHandlerService<(), (i32,)> for UpService {
    async fn call_request_handler_service<R: Read, W: ResponseWriter<Error = R::Error>>(
        &self,
        _state: &(),
        (steps,): (i32,),
        request: Request<'_, R>,
        response_writer: W,
    ) -> Result<ResponseSent, W::Error> {
        up_response(self.state, steps)
            .write_to(request.body_connection.finalize().await?, response_writer)
            .await
    }
}

impl RequestHandlerService<(), (i32,)> for DownService {
    async fn call_request_handler_service<R: Read, W: ResponseWriter<Error = R::Error>>(
        &self,
        _state: &(),
        (steps,): (i32,),
        request: Request<'_, R>,
        response_writer: W,
    ) -> Result<ResponseSent, W::Error> {
        down_response(self.state, steps)
            .write_to(request.body_connection.finalize().await?, response_writer)
            .await
    }
}

impl RequestHandlerService<(), (i32,)> for MoveToService {
    async fn call_request_handler_service<R: Read, W: ResponseWriter<Error = R::Error>>(
        &self,
        _state: &(),
        (position,): (i32,),
        request: Request<'_, R>,
        response_writer: W,
    ) -> Result<ResponseSent, W::Error> {
        move_to_response(self.state, position)
            .write_to(request.body_connection.finalize().await?, response_writer)
            .await
    }
}

pub struct Application {
    state: &'static DeskControllerState,
}

impl AppBuilder for Application {
    type PathRouter = impl routing::PathRouter;

    fn build_app(self) -> Router<Self::PathRouter> {
        Router::new()
            .route("/", routing::get(index))
            .route(
                "/home",
                routing::post_service(HomeService { state: self.state }),
            )
            .route(
                ("/up", parse_path_segment::<i32>()),
                routing::post_service(UpService { state: self.state }),
            )
            .route(
                ("/down", parse_path_segment::<i32>()),
                routing::post_service(DownService { state: self.state }),
            )
            .route(
                ("/move", parse_path_segment::<i32>()),
                routing::post_service(MoveToService { state: self.state }),
            )
    }
}

pub const WEB_TASK_POOL_SIZE: usize = 2;

#[embassy_executor::task(pool_size = WEB_TASK_POOL_SIZE)]
pub async fn web_task(
    id: usize,
    stack: Stack<'static>,
    router: &'static AppRouter<Application>,
    config: &'static picoserve::Config,
) -> ! {
    let port = 80;
    let mut tcp_rx_buffer = [0; 1024];
    let mut tcp_tx_buffer = [0; 1024];
    let mut http_buffer = [0; 2048];

    picoserve::Server::new(router, config, &mut http_buffer)
        .listen_and_serve(id, stack, port, &mut tcp_rx_buffer, &mut tcp_tx_buffer)
        .await
        .into_never()
}

pub struct WebApp {
    pub router: &'static Router<<Application as AppBuilder>::PathRouter>,
    pub config: &'static picoserve::Config,
    pub state: &'static DeskControllerState,
}

impl WebApp {
    pub fn new(state: &'static DeskControllerState) -> Self {
        let router =
            picoserve::make_static!(AppRouter<Application>, Application { state }.build_app());
        let config = picoserve::make_static!(
            picoserve::Config,
            picoserve::Config::new(Default::default()).keep_connection_alive()
        );

        Self {
            router,
            config,
            state,
        }
    }
}
